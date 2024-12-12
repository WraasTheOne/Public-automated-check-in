package services

import (
	"context"
	"database/sql"
	"fmt"

	"Public-automated-check-in/calc/internal/db"
	"Public-automated-check-in/calc/proto"
)

type Calctripservice struct {
	proto.UnimplementedAuthServiceServer
}

func (s *Calctripservice) AuthClient(ctx context.Context, req *proto.CalcTripRequest) (*proto.CalcTripResponse, error) {

	tripid := req.GetTripid()
	if tripid == 0 {
		return &proto.CalcTripResponse{
			Status:  false,
			Message: "Invalid tripid",
		}, fmt.Errorf("no tripid provided")
	}

	tripdata, err := db.GetTripData(tripid)
	if err != nil {
		return &proto.CalcTripResponse{
			Status:  false,
			Message: "Failed to get trip data",
		}, fmt.Errorf("failed to get trip data: %v", err)
	}
	start, end, err := StartandEndLocation(tripdata)
	if err != nil {
		return &proto.CalcTripResponse{
			Status:  false,
			Message: "Failed to get start and end location",
		}, fmt.Errorf("failed to get start and end location: %v", err)
	}

	if err := db.UpdateTripLocation(tripid, start, end); err != nil {
		return &proto.CalcTripResponse{
			Status:  false,
			Message: "Failed to update trip location",
		}, fmt.Errorf("failed to update trip location: %v", err)
	}

	return &proto.CalcTripResponse{
		Status:  true,
		Message: "Trip location updated successfully",
	}, nil

}

func StartandEndLocation(tripdata *sql.Rows) (string, string, error) {
	var startLocation, endLocation string
	var position string
	var timestamp string // Assuming the timestamp column is of string type; adjust if necessary

	firstRow := true

	// Iterate through rows to find the start and end locations
	for tripdata.Next() {
		// Scan the row into position and timestamp variables
		if err := tripdata.Scan(&position, &timestamp); err != nil {
			return "", "", fmt.Errorf("failed to scan trip data: %v", err)
		}

		if firstRow {
			// First row is the start location
			startLocation = position
			firstRow = false
		}
		// Continuously update endLocation with the latest row
		endLocation = position
	}

	// Check for errors during iteration
	if err := tripdata.Err(); err != nil {
		return "", "", fmt.Errorf("error iterating trip data rows: %v", err)
	}

	return startLocation, endLocation, nil
}
