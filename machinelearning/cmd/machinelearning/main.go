package main

import (
	"log"
	"net"

	service "Public-automated-check-in/machinelearning/internal/services"
	"Public-automated-check-in/machinelearning/proto"

	"google.golang.org/grpc"
)

func main() {
	listener, err := net.Listen("tcp", ":3002")
	if err != nil {
		log.Fatalf("Failed to listen: %v", err)
	}

	grpcServer := grpc.NewServer()
	proto.RegisterMachineLearningServiceServer(grpcServer, &service.MachineLearningService{})

	log.Println("gRPC server is running on port 3002")
	if err := grpcServer.Serve(listener); err != nil {
		log.Fatalf("Failed to serve: %v", err)
	}
}
