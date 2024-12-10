package util

import (
	"fmt"
	"log"
	"net"
	"os"

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

func GetRedisEnvs() (string, string, string) {
	var host = LoadEnv("REDIS_HOST")
	var port = LoadEnv("REDIS_PORT")
	var password = LoadEnv("REDIS_PASSWORD")
	return host, port, password
}
