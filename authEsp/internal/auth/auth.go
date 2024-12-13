package auth


import (
	"Public-automated-check-in/authEsp/proto"
	"context"
	"log"
	"net/http"

	"github.com/gin-gonic/gin"
	"google.golang.org/grpc"
)


// make grpc call for auth service using roken from incomming request
func AuthMiddleware(c *gin.Context) {
	token := c.GetHeader("Authorization")
	if token == "" {
		c.JSON(http.StatusUnauthorized, gin.H{"error": "Authorization token is required"})
		c.Abort()
		return
	}

	conn, err := grpc.Dial("localhost:3002", grpc.WithInsecure())
	if err != nil { 
		log.Printf("Failed to connect to AuthService: %v", err)
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to connect to AuthService"})
		c.Abort()
		return
	}
	defer conn.Close()

	client := proto.NewAuthServiceClient(conn)
	response, err := client.AuthClient(context.Background(), &proto.AuthClientRequest{Token: token})
	if err != nil {
		log.Printf("Failed to authenticate token: %v", err)
		c.JSON(http.StatusUnauthorized, gin.H{"error": "Authentication failed"})
		c.Abort()
		return
	}

	if !response.AuthStatus {
		c.JSON(http.StatusUnauthorized, gin.H{"error": response.Message})
		c.Abort()
		return
	}

	c.Next()
}

