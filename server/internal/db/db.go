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

// CloseDB closes the database connection
func CloseDB() {
	if db != nil {
		db.Close()
	}
}
