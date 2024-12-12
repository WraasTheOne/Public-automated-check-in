package main

import (
	"log"
	"net"

	service "Public-automated-check-in/calc/internal/services"
	"Public-automated-check-in/calc/proto"

	"google.golang.org/grpc"
)

func main() {
	// Start the gRPC server
	listener, err := net.Listen("tcp", ":3003")
	if err != nil {
		log.Fatalf("Failed to listen: %v", err)
	}

	grpcServer := grpc.NewServer()
	proto.RegisterAuthServiceServer(grpcServer, &service.Calctripservice{})

	log.Println("gRPC server is running on port 3002")
	if err := grpcServer.Serve(listener); err != nil {
		log.Fatalf("Failed to serve: %v", err)
	}
}
