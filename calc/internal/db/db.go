package db

import (
	"database/sql"
	"fmt"
	"log"

	_ "github.com/go-sql-driver/mysql"
)

var db *sql.DB

// InitDB initializes the MySQL database connection
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

// get trip_data from trip id
func GetTripData(tripID int64) (*sql.Rows, error) {

	var Query = "SELECT * FROM trip_data WHERE trip_id = ?"
	result, err := db.Query(Query, tripID)
	if err != nil {
		return nil, fmt.Errorf("failed to get trip data: %v", err)
	}
	return result, nil

}

// updatestart_location and end_location in trip with trip id
func UpdateTripLocation(tripID int64, startLocation, endLocation string) error {
	_, err := db.Exec("UPDATE trip SET start_location = ?, end_location = ? WHERE trip_id = ?", startLocation, endLocation, tripID)
	if err != nil {
		return fmt.Errorf("failed to update trip location: %v", err)
	}
	return nil
}

func CloseDB() {
	if db != nil {
		db.Close()
	}
}
