package main

import (
	"Public-automated-check-in/server/internal/services"
	"log"

	"github.com/gin-gonic/gin"
)

func main() {
	router := gin.Default()
	// Define routes for login and register
	router.POST("/login", services.LoginHandler)
	router.POST("/register", services.RegisterHandler)
	router.GET("/checkInStatus", services.GetCheckinStatusHandler)
	router.GET("/userjourneys", services.GetJourneysHandler)

	log.Println("Starting server on :8080...")
	if err := router.Run(":8080"); err != nil {
		log.Fatalf("Failed to start server: %v", err)
	}
}
