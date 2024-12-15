package services

import (
	"Public-automated-check-in/server/internal/db"
	"Public-automated-check-in/server/internal/redisdb"
	"Public-automated-check-in/server/pkg/hash"
	"Public-automated-check-in/server/pkg/util"
	"crypto/sha256"
	"database/sql"
	"fmt"
	"net/http"
	"strings"

	"github.com/gin-gonic/gin"
)

// LoginHandler handles the /login endpoint
func LoginHandler(c *gin.Context) {
	var req struct {
		Username string `json:"username" binding:"required"`
		Password string `json:"password" binding:"required"`
	}

	// Bind JSON payload to request struct
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"message": "Invalid request payload"})
		return
	}

	username := req.Username
	incpassword := req.Password

	// Load environment variables for database configuration
	var usernamedb, passworddb, hostdb, portdb, dbname = util.GetDbEnvs()

	// Initialize database connection
	if err := db.InitDB(usernamedb, passworddb, hostdb, portdb, dbname); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"message": "Failed to connect to database"})
		return
	}

	defer db.CloseDB()


	// Fetch user details
	id, password, err := db.GetUser(username)
	if err != nil {
		if err == sql.ErrNoRows {
			c.JSON(http.StatusUnauthorized, gin.H{"message": "Invalid username or password"})
			return
		}
		c.JSON(http.StatusInternalServerError, gin.H{"message": "Failed to fetch user from database"})
		return
	}

	if !hash.ComparePasswords(password, incpassword) {
		c.JSON(http.StatusUnauthorized, gin.H{"message": "Invalid username or password"})
		return
	}

	// Create a new session
	session, err := util.CreateSession(id)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"message": "Failed to create session"})
		return
	}

	redisdb.InitRedis()
	defer redisdb.CloseRedis()
	redisdb.SetWithExpiration(session.Token, id, 24*3600)//how long is 3600?: 3600 seconds = 1 hourc 

	c.JSON(http.StatusOK, gin.H{
		"message": "Login successful",
		"token":   session.Token,
	})
}

// RegisterHandler handles the /register endpoint
func RegisterHandler(c *gin.Context) {
	var req struct {
		Username string `json:"username" binding:"required"`
		Password string `json:"password" binding:"required"`
	}

	// Bind JSON payload to request struct
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"message": "Invalid request payload"})
		return
	}

	username := req.Username
	rawPassword := req.Password

	// Hash the password using SHA-256
	hashedPassword := sha256.Sum256([]byte(rawPassword))
	hashedPasswordStr := fmt.Sprintf("%x", hashedPassword)

	// Load environment variables for database configuration
	var usernamedb, passworddb, hostdb, portdb, dbname = util.GetDbEnvs()

	// Initialize database connection
	if err := db.InitDB(usernamedb, passworddb, hostdb, portdb, dbname); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"message": "Failed to connect to database"})
		return
	}
	defer db.CloseDB()

	// Register the user
	userID, err := db.RegisterUser(username, hashedPasswordStr)
	if err != nil {
		if strings.Contains(err.Error(), "Duplicate entry") {
			c.JSON(http.StatusConflict, gin.H{"message": "Username already exists"})
			return
		}
		c.JSON(http.StatusInternalServerError, gin.H{"message": "Failed to register user"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"userId":  userID,
		"message": "Registration successful",
	})
}
