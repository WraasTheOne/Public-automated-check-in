package main

import (
	"log"
	"net"

	service "Public-automated-check-in/authserver/internal/services"
	"Public-automated-check-in/authserver/proto"

	"google.golang.org/grpc"
)

func main() {
	// Start the gRPC server
	listener, err := net.Listen("tcp", ":3002")
	if err != nil {
		log.Fatalf("Failed to listen: %v", err)
	}

	grpcServer := grpc.NewServer()
	proto.RegisterAuthServiceServer(grpcServer, &service.AuthClientService{})

	log.Println("gRPC server is running on port 3002")
	if err := grpcServer.Serve(listener); err != nil {
		log.Fatalf("Failed to serve: %v", err)
	}
}
