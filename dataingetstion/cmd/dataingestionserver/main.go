package main

import (
	"Public-automated-check-in/dataingestion/internal/auth"
	"Public-automated-check-in/dataingestion/internal/services"
	"log"

	"github.com/gin-gonic/gin"
)

func main() {

	// Setup Gin server
	router := gin.Default()

	// Middleware to authenticate token
	router.Use(auth.AuthMiddleware)

	// WebSocket endpoint
	router.GET("/dataingestion", services.WebSocketHandler)

	// Start the server
	port := "8081"
	log.Printf("WebSocket ingestion service running on port %s", port)
	if err := router.Run("0.0.0.0:" + port); err != nil {
		log.Fatalf("Failed to start server: %v", err)
	}
}
