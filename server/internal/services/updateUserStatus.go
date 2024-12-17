package services

import (
	"Public-automated-check-in/server/internal/db"
	"Public-automated-check-in/server/internal/redisdb"
	"Public-automated-check-in/server/pkg/util"
	"database/sql"
	"fmt"
	"net/http"

	"github.com/gin-gonic/gin"
)

func GetCheckinStatusHandler(c *gin.Context) {
//get the token from the header
	token := c.GetHeader("Authorization")
	fmt.Println("the token is ", token)

	if token == "" {
		c.JSON(http.StatusUnauthorized, gin.H{"message": "Unauthorized"})
		return
	}

	if err := redisdb.InitRedis(); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"message": "Failed to connect to Redis"})
		return
	}

	userId, err := redisdb.GetUserId(token)
	if err != nil {
		fmt.Println("the error is ", err)
		c.JSON(http.StatusUnauthorized, gin.H{"message": "isnterhere herher"})
		return
	}

	// Load environment variables for database configuration
	var usernamedb, passworddb, hostdb, portdb, dbname = util.GetDbEnvs()
	fmt.Println("the db envs are ", usernamedb, passworddb, hostdb, portdb, dbname)

	// Initialize database connection
	if err := db.InitDB(usernamedb, passworddb, hostdb, portdb, dbname); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"message": "Failed to connect to database"})
		return
	}
	defer db.CloseDB()

	// Fetch user checkin status
	isCheckedIn, walletstauts, err := db.GetCheckinStatus(userId)
	if err != nil {
		if err == sql.ErrNoRows {
			// User has not checked in
			c.JSON(http.StatusOK, gin.H{"checked_in": isCheckedIn, "wallet": walletstauts})
			return
		}
		c.JSON(http.StatusInternalServerError, gin.H{"message": "Failed to fetch user checkin status"})
		return
	}
	// User has checked in
	c.JSON(http.StatusOK, gin.H{"checked_in": isCheckedIn, "wallet": walletstauts})
}


func GetJourneysHandler(c *gin.Context) {
	//get the token from the header
	token := c.GetHeader("Authorization")
	fmt.Println("the token is ", token)
	if token == "" {
		c.JSON(http.StatusUnauthorized, gin.H{"message": "Unauthorized"})
		return
	}

	if err := redisdb.InitRedis(); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"message": "Failed to connect to Redis"})
		return
	}

	userId, err := redisdb.GetUserId(token)
	if err != nil {
		fmt.Println("the error is ", err)
		c.JSON(http.StatusUnauthorized, gin.H{"message": "isnterhere herher"})
		return
	}

	var usernamedb, passworddb, hostdb, portdb, dbname = util.GetDbEnvs()

	if err := db.InitDB(usernamedb, passworddb, hostdb, portdb, dbname); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"message": "Failed to connect to database"})
		return
	}
	defer db.CloseDB()

	journeys, err := db.GetJourneys(userId)
	if err != nil {
		if err == sql.ErrNoRows {
			// No journeys found so 404
			c.JSON(http.StatusNotFound, gin.H{"message": "No journeys found"})
			return
		}
		c.JSON(http.StatusInternalServerError, gin.H{"message": "Unable to fetch journeys"})
		return
	}

	var journeysList []interface{}
	for _, journey := range journeys {
		journeysList = append(journeysList, gin.H{
			"start_location": journey.StartLocation,
			"end_location":   journey.EndLocation,
			"price":          journey.Price,
			"trip_date":      journey.TripDate,
		})
	}

	c.JSON(http.StatusOK, gin.H{"journeys": journeysList})

}


