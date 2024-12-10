package auth

import (
	"Public-automated-check-in/dataingestion/proto"
	"log"

	"github.com/gin-gonic/gin"
	"google.golang.org/grpc"
)

// make grpc call for auth service using roken from incomming request
func AuthClient(c *gin.Context) bool {
	// Set up a connection to the server.
	conn, err := grpc.NewClient("localhost:50051", grpc.WithInsecure())
	if err != nil {
		log.Fatalf("did not connect: %v", err)
	}

	client := proto.NewAuthServiceClient(conn)

}
