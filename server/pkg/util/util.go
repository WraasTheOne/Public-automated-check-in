package util

import (
	"crypto/sha256"
	"fmt"
	"log"
	"net"
	"os"
	"time"

	"github.com/joho/godotenv"
)

func LoadEnv(variable string) string {

	err := godotenv.Load("./config/.env")
	if err != nil {
		log.Fatalf("Error loading .env file: %v", err)
	}
	port := os.Getenv(variable)
	if port == "" {
		log.Fatal(variable + " is not set in the environment")
	}
	return port
}

func StartServer(port string) net.Listener {
	fmt.Println("Starting server on port " + port)
	listener, err := net.Listen("tcp", port)
	if err != nil {
		log.Fatal(err)
	}
	fmt.Println("Server is running...")
	return listener

}

type Session struct {
	Token  string
	UserID int64
}

func CreateSession(userID int64) (*Session, error) {
	// Generate a unique token (e.g., UUID or random string)
	token := fmt.Sprintf("%x", sha256.Sum256([]byte(fmt.Sprintf("%d:%d", userID, time.Now().UnixNano()))))

	return &Session{
		Token:  token,
		UserID: userID,
	}, nil
}

func GetDbEnvs() (string, string, string, string, string) {
	var usernamedb = LoadEnv("USERDB_USER")
	var passworddb = LoadEnv("USERDB_PASSWORD")
	var hostdb = LoadEnv("USERDB_HOST")
	var portdb = LoadEnv("USERDB_PORT")
	var dbname = LoadEnv("USERDB_DBNAME")
	return usernamedb, passworddb, hostdb, portdb, dbname
}

func GetRedisEnvs() (string, string, string) {
	var host = LoadEnv("REDIS_HOST")
	var port = LoadEnv("REDIS_PORT")
	var password = LoadEnv("REDIS_PASSWORD")
	return host, port, password
}
