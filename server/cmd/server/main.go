package main

import (
	"log"
	"net"

	service "Public-automated-check-in/server/internal/services"
	"Public-automated-check-in/server/proto"

	"google.golang.org/grpc"
)

func main() {
	// Start the gRPC server
	listener, err := net.Listen("tcp", ":3001")
	if err != nil {
		log.Fatalf("Failed to listen: %v", err)
	}

	grpcServer := grpc.NewServer()
	proto.RegisterLoginServiceServer(grpcServer, &service.LoginService{})

	log.Println("gRPC server is running on port 3001")
	if err := grpcServer.Serve(listener); err != nil {
		log.Fatalf("Failed to serve: %v", err)
	}
}
