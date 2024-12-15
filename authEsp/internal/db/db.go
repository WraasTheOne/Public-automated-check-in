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



//mysql> describe bluetooth_devices;
//+---------------+--------------+------+-----+-------------------+-------------------+
//| Field         | Type         | Null | Key | Default           | Extra             |
//+---------------+--------------+------+-----+-------------------+-------------------+
//| id            | int          | NO   | PRI | NULL              | auto_increment    |
//| device_name   | varchar(255) | YES  |     | NULL              |                   |
//| registered_at | datetime     | YES  |     | CURRENT_TIMESTAMP | DEFAULT_GENERATED |
//| vehicle_id    | int          | NO   | MUL | NULL              |                   |
//| secret        | varchar(255) | YES  |     | NULL              |                   |
//+---------------+--------------+------+-----+-------------------+-------------------+

func GetSecretKey(espName string) (string, error) {
	var secretKey string
	query := fmt.Sprintf("SELECT secret FROM bluetooth_devices WHERE device_id_name = '%s'", espName)
	// if no key is found, return an empty string thne we now taht the device is not registered
	err := db.QueryRow(query).Scan(&secretKey)
	if err != nil {
		return "", err
	}
	return secretKey, nil

}

func CloseDB() {
	if db != nil {
		db.Close()
	}
}
