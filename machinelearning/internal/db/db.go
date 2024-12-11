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
	query := "INSERT INTO trip_data (trip_id, data) VALUES (?, ?)"
	_, err := db.Exec(query, tripID, position)
	if err != nil {
		return fmt.Errorf("failed to insert trip data: %v", err)
	}

	return nil
}
