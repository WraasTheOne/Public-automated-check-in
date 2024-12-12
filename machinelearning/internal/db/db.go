package db

import (
	"database/sql"
	"fmt"
	"log"

	_ "github.com/go-sql-driver/mysql"
)

var db *sql.DB

func InitDB(username, password, host, port, dbName string) error {
	dsn := fmt.Sprintf("%s:%s@tcp(%s:%s)/%s?parseTime=true", username, password, host, port, dbName)
	var err error
	db, err = sql.Open("mysql", dsn)
	if err != nil {
		return fmt.Errorf("failed to connect to MySQL: %v", err)
	}

	// Test the connection
	if err := db.Ping(); err != nil {
		return fmt.Errorf("failed to ping MySQL: %v", err)
	}

	log.Println("Connected to MySQL successfully")
	return nil
}

func CloseDB() {
	if db != nil {
		db.Close()
	}
}

func RegisterTrip(userID int64) (int64, error) {
	query := "INSERT INTO trips (user_id) VALUES (?)"
	data, err := db.Exec(query, userID)
	if err != nil {
		return 0, fmt.Errorf("failed to insert trip: %v", err)
	}
	lastInsertId, err := data.LastInsertId()
	if err != nil {
		return 0, fmt.Errorf("failed to get last insert id: %v", err)
	}

	return lastInsertId, nil
}

func RegisterTripData(tripID int64, position string) error {
	query := "INSERT INTO trip_data (trip_id, postion) VALUES (?, ?)"
	_, err := db.Exec(query, tripID, position)
	if err != nil {
		return fmt.Errorf("failed to insert trip data: %v", err)
	}

	return nil
}

func GetCurrentLocation(bluetoothDeviceID int) (string, int, error) {
	// SQL query to fetch the current location
	query := `
		SELECT v.current_location, v.
		FROM vehicle v
		INNER JOIN bluetooth_devices bd ON v.id = bd.vehicle_id
		WHERE bd.id = ?;
	`

	// Execute the query
	var currentLocation string
	var ammount_of_passengers int
	err := db.QueryRow(query, bluetoothDeviceID).Scan(&currentLocation, &ammount_of_passengers)
	if err != nil {
		if err == sql.ErrNoRows {
			// No rows found
			return "", 0, fmt.Errorf("no vehicle found for bluetooth_device_id %d", bluetoothDeviceID)
		}
		// Other errors
		return "", 0, err
	}

	// Return the current location
	return currentLocation, ammount_of_passengers, nil
}

func AddPassangerToVehicle(bluetoothDeviceID int) error {
	// SQL query to add a passenger to the vehicle
	query := `
		UPDATE vehicle
		SET ammount_of_passengers = ammount_of_passengers + 1
		WHERE id = (
			SELECT vehicle_id
			FROM bluetooth_devices
			WHERE id = ?
		);
	`

	// Execute the query
	_, err := db.Exec(query, bluetoothDeviceID)
	if err != nil {
		return fmt.Errorf("failed to add passenger to vehicle: %v", err)
	}

	return nil
}
