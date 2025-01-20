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

// set end "SetEndLocationinTrip" location of the trip
func SetEndLocationinTrip(userID int64, location string) error {
	query := `update trips set end_location = ? where user_id = ? order by trip_date desc limit 1`
	_, err := db.Exec(query, location, userID)
	if err != nil {
		log.Println("Failed to set end location:", err)
		return err
	}
	return nil
}


func GetFinishedUsers() []int64 {
	query := "SELECT id FROM users WHERE is_checked_in = 1 AND TIMESTAMPDIFF(MINUTE, in_transport_updated_at, NOW()) > 2;"
	var id int64 
	err := db.QueryRow(query).Scan(&id)
	if err != nil {
		log.Println("no users to update", err)
		return nil
	}

	query = "UPDATE users SET in_transport= 0, is_checked_in = 0 WHERE id = ?"
	_, err = db.Exec(query, id)
	if err != nil {
		log.Println("Failed to update user:", err)
		return nil 
	}

	return []int64{id}
}


func GetAndSetLastLocation(userID int64)(int64, error) {
	//first we need to get the trip id of the 
	//last trip of the user
	query := `select id from trips where user_id = ? and finished_trip = 0 order by trip_date desc limit 1`
	var tripID int64 
	err := db.QueryRow(query, userID).Scan(&tripID)
	if err != nil {
		log.Println("Failed to get trip ID:", err)
		return 0, err
	}
	//then we need to get the last location of the trip
	query = "select position from trip_data where trip_id = ? order by time_stamp desc limit 1"
	var position string 
	err = db.QueryRow(query, tripID).Scan(&position)
	if err != nil {
		log.Println("Failed to get last location:", err)
		return 0, err
	}
	// set the end location of the trip
	err = SetEndLocationinTrip(userID, position)
	if err != nil {
		log.Println("Failed to set end location")
		return 0, err
	}
	return tripID, nil
}

// GetLocations fetches all locations of a user
func GetLocations(userID int64, tripID int64) ([]string, error) {
	query := "SELECT position FROM trip_data WHERE trip_id = ?"
	rows, err := db.Query(query, tripID)
	if err != nil {
		log.Println("Failed to get locations:", err)
		return nil, err
	}
	defer rows.Close()

	var locations []string
	for rows.Next() {
		var location string
		if err := rows.Scan(&location); err != nil {
			log.Println("Failed to scan location:", err)
			return nil, err
		}
		locations = append(locations, location)
	}

	return locations, nil
}
func UpdateTripPrice(tripID int64, price int) error {
	query := "UPDATE trips SET price = ? WHERE id = ?"
	_, err := db.Exec(query, price, tripID)
	if err != nil {
		log.Println("Failed to update trip price:", err)
		return err
	}
	return nil
}

func UpdateUserWallet(userID int64, price int) error {
	query := "UPDATE users SET wallet = wallet - ? WHERE id = ?"
	_, err := db.Exec(query, price, userID)
	if err != nil {
		log.Println("Failed to update user wallet balance:", err)
	}
	return err
}



func CloseDB() {
	if db != nil {
		db.Close()
	}
}
