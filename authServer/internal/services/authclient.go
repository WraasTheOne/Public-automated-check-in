package services

import (
	"context"
	"fmt"

	redisdb "Public-automated-check-in/authserver/internal/redisdb"
	"Public-automated-check-in/authserver/proto"
)

type AuthClientService struct {
	proto.UnimplementedAuthServiceServer
}

func (s *AuthClientService) Login(ctx context.Context, req *proto.AuthClientRequest) (*proto.AuthClientResponse, error) {

	token := req.GetToken()

	if token == "" {
		return &proto.AuthClientResponse{
			Message: "Invalid token",
		}, nil
	}

	// Initialize Redis client
	if err := redisdb.InitRedis(); err != nil {
		return nil, fmt.Errorf("failed to connect to Redis: %v", err)
	}

	// Check if the token exists in Redis
	var authStatus = redisdb.CheckToken(token)
	if !authStatus {
		return &proto.AuthClientResponse{
			Message: "Invalid token",
		}, nil
	}

	// Close the Redis connection
	defer redisdb.CloseRedis()

	return &proto.AuthClientResponse{
		AuthStatus: true,
		Message:    "Token is valid",
	}, nil

}
