package main


import (
	"Public-automated-check-in/authEsp/internal/auth"
	"Public-automated-check-in/authEsp/internal/services"
	"log"
	"github.com/gin-gonic/gin"
)

func main() {

	// Setup Gin server
	router := gin.Default()

	// Middleware to authenticate token
	router.Use(auth.AuthMiddleware)

//	router.GET("/esp/{id}/challenge", futnisn to a challenge for the esp
	router.GET("/getChallenge/esp/:esp_name", services.GetChallengeForEsp)
	router.POST("/verifyChallenge", services.AuthenticateEsp)



	port := "8050"
	log.Printf("WebSocket ingestion service running on port %s", port)
	if err := router.Run(":" + port); err != nil {
		log.Fatalf("Failed to start server: %v", err)
	}
}

