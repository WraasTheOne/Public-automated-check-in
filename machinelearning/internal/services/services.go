package services

import (
	"Public-automated-check-in/machinelearning/internal/db"
	"Public-automated-check-in/machinelearning/internal/redisdb"
	"Public-automated-check-in/machinelearning/proto"
	"fmt"
	"time"

	"context"
)

type MachineLearningService struct {
	proto.UnimplementedMachineLearningServiceServer
}

func (s *MachineLearningService) Predict(ctx context.Context, req *proto.PredictRequest) (*proto.PredictResponse, error) {
	userid := int64(req.GetUserID())

	// Step 1: Connect to the database and initialize
	err := db.InitDB("root", "rootpassword", "127.0.0.1", "3307", "userdb")
	if err != nil {
		return &proto.PredictResponse{
			ServiceStatus: false,
			Message:       "Failed to connect to MySQL",
		}, nil
	}
	defer db.CloseDB()

	// Step 2: Create a trip and save trip ID
	tripID, err := db.RegisterTrip(userid)
	if err != nil {
		fmt.Printf("Error: %v", err)
		return &proto.PredictResponse{
			ServiceStatus: false,
			Message:       "Failed to register trip",
		}, nil
	}

	// Step 3: Connect to Redis
	err = redisdb.InitRedis()
	if err != nil {
		return &proto.PredictResponse{
			ServiceStatus: false,
			Message:       "Failed to connect to Redis",
		}, nil
	}
	defer redisdb.CloseRedis()

	// Step 4: Start loop to get info from Redis stream
	token := req.GetToken() // Replace with dynamic token if needed
	timeout := time.After(30 * time.Second)
	reset := make(chan bool)
	go func() {
		ticker := time.NewTicker(30 * time.Second)
		for {
			select {
			case <-ticker.C:
				reset <- true
			case <-reset:
				ticker.Reset(30 * time.Second)
			}
		}
	}()

	for {
		select {
		case <-timeout:
			// End loop after 30 seconds without data
			fmt.Println("Timeout reached, ending loop")
			return &proto.PredictResponse{
				ServiceStatus: true,
				Message:       "Loop ended after 30 seconds",
			}, nil
		default:
			// Step 5: Get data from Redis stream
			values, messageID, err := redisdb.ReadFirstFromStream(token)
			if err != nil {
				return &proto.PredictResponse{
					ServiceStatus: false,
					Message:       fmt.Sprintf("Failed to read data from Redis: %v", err),
				}, nil
			}

			// Reset timeout counter when data comes in
			reset <- true

			// Step 6: Get vehicle location
			location, err := getVehicleLocation()
			if err != nil {
				return &proto.PredictResponse{
					ServiceStatus: false,
					Message:       fmt.Sprintf("Failed to get vehicle location: %v", err),
				}, nil
			}

			// Step 7: Store trip data
			err = storeTripData(tripID, location)
			if err != nil {
				return &proto.PredictResponse{
					ServiceStatus: false,
					Message:       fmt.Sprintf("Failed to store trip data: %v", err),
				}, nil
			}

			// Step 8: Delete message from Redis stream
			err = redisdb.DeleteMessageFromStream(token, messageID)
			if err != nil {
				return &proto.PredictResponse{
					ServiceStatus: false,
					Message:       fmt.Sprintf("Failed to delete message from Redis: %v", err),
				}, nil
			}
		}
	}
}
