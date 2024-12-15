package db

import (
	"database/sql"
	"fmt"
	"log"

	_ "github.com/go-sql-driver/mysql"
)

var db *sql.DB

type Journey struct {
	StartLocation string `json:"start_location"`
	EndLocation   string `json:"end_location"`
	Price         int    `json:"price"`
	TripDate      string `json:"trip_date"`
}

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

// RegisterUser registers a new user in the database
func RegisterUser(username, password string) (int64, error) {
	query := "INSERT INTO users (username, password) VALUES (?, ?)"
	result, err := db.Exec(query, username, password)
	if err != nil {
		return 0, fmt.Errorf("failed to insert user: %v", err)
	}

	userID, err := result.LastInsertId()
	if err != nil {
		return 0, fmt.Errorf("failed to get last insert ID: %v", err)
	}

	return userID, nil
}

// GetUser fetches a user by username
func GetUser(username string) (int64, string, error) {
	query := "SELECT id, password FROM users WHERE username = ?"
	var id int64
	var password string
	err := db.QueryRow(query, username).Scan(&id, &password)
	if err == sql.ErrNoRows {
		return 0, "", nil // No user found
	}
	if err != nil {
		return 0, "", fmt.Errorf("failed to query user: %v", err)
	}

	return id, password, nil
}

func GetCheckinStatus(id string) (bool, int64, error) {
	query := "SELECT is_checked_in, wallet FROM users WHERE id = ?"
	var checkedIn bool
	var o int64
	err := db.QueryRow(query, id).Scan(&checkedIn, &o)
	if err == sql.ErrNoRows {
		return false, o, nil // No user found
	}
	if err != nil {
		return false, 0, fmt.Errorf("failed to query user: %v", err)
	}

	return checkedIn, o, nil
}


func GetJourneys(id string) ([]Journey, error) {
	query := "SELECT start_location, end_location, price, trip_date FROM trips WHERE user_id = ? and end_location is not NULL"
	rows, err := db.Query(query, id)
	if err != nil {
		return nil, fmt.Errorf("failed to query trips: %v", err)
	}
	defer rows.Close()

	var journeys []Journey
	for rows.Next() {
		var journey Journey
		if err := rows.Scan(&journey.StartLocation, &journey.EndLocation, &journey.Price, &journey.TripDate); err != nil {
			return nil, fmt.Errorf("failed to scan row: %v", err)
		}
		journeys = append(journeys, journey)
	}

	if err := rows.Err(); err != nil {
		return nil, fmt.Errorf("failed to iterate rows: %v", err)
	}

	return journeys, nil
}

// CloseDB closes the database connection
func CloseDB() {
	if db != nil {
		db.Close()
	}
}
