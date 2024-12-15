package services

import (

	"Public-automated-check-in/authEsp/internal/redisdb"
	"net/http"
	"fmt"
	"Public-automated-check-in/authEsp/internal/db"
	"github.com/gin-gonic/gin"
	"Public-automated-check-in/authEsp/pkg/util"
)

func GetChallengeForEsp(c *gin.Context) {
	token := c.GetHeader("Authorization")
	if token == "" {
		c.JSON(http.StatusUnauthorized, gin.H{"error": "Authorization token is required"})
		c.Abort()
		return
	}

	name := c.Param("esp_name")
	if name == "" {
		c.JSON(http.StatusBadRequest, gin.H{"error": "ESP name is required"})
		c.Abort()
		return
	}


	redisdb.InitRedis()
	defer redisdb.CloseRedis()


	var usernamedb, passworddb, hostdb, portdb, dbname = util.GetDbEnvs()
	defer db.CloseDB()
	if err := db.InitDB(usernamedb, passworddb, hostdb, portdb, dbname); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to connect to database"})
		c.Abort()
		return
	}

	secrectKey, err := db.GetSecretKey(name)
	//if error then the esp not regerterd
	if err != nil {
		c.JSON(http.StatusUnauthorized, gin.H{"error": "ESP not registered"})
		c.Abort()
		return
	}

	challenge, err := util.GenerateChallenge(16)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to generate challenge"})
		c.Abort()
		return
	}
//								 NOTE:
//          the challenge key is the:    "token + challenge"  that are hashed with sha256
	challengeKey := util.GenerateChallengeKey(token, challenge)
	hmacValueForlater := util.HashMsg(challenge, secrectKey)

	fmt.Println("hmacValueForlater: ", hmacValueForlater)

	redisdb.InitRedis()
	defer redisdb.CloseRedis()

	err = redisdb.StoreChallengeForEsp(challengeKey, hmacValueForlater)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to store challenge"})
		c.Abort()
		return
	}

	fmt.Println("challengeKey: ", challengeKey)
	fmt.Println("hmacValueForlater: ", hmacValueForlater)
	fmt.Println("secrectKey: ", secrectKey)
	fmt.Println("challenge: ", challenge)

	c.JSON(http.StatusOK, gin.H{"challenge": challenge})

}


//AuthenticateEsp:_

func AuthenticateEsp(c *gin.Context) {
	token := c.GetHeader("Authorization")
	fmt.Println("token: ", token)
	if token == "" {
		c.JSON(http.StatusUnauthorized, gin.H{"error": "Authorization token is required"})
		c.Abort()
		return
	}


	var json struct {
		Challenge string `json:"challenge"`
		ComputedHmac string `json:"computed_hmac"`

		}

	if err := c.ShouldBindJSON(&json); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": "Invalid request"})
		c.Abort()
		return
	}

	challengeKey := util.GenerateChallengeKey(token, json.Challenge)

	redisdb.InitRedis()
	defer redisdb.CloseRedis()

	hmacValue, err := redisdb.MatchChallengeForEsp(challengeKey)
	if err != nil {
		c.JSON(http.StatusUnauthorized, gin.H{"error": "Invalid challenge"})
		c.Abort()
		return
	}

	if hmacValue != json.ComputedHmac {
		c.JSON(http.StatusUnauthorized, gin.H{"error": "Invalid HMAC"})
		c.Abort()
		return
	}

	c.JSON(http.StatusOK, gin.H{"message": "ESP authenticated successfully"})

}

