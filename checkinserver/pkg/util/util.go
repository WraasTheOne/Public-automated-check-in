package util

import (
	"log"
	"os"
	"github.com/joho/godotenv"

)

func GetPrice(locations []string) int {

	//for each unique location, add to the zones
	zone := 0
	//filter the unique locations
	uniqueLocations := make(map[string]bool)
	for _, location := range locations {
		uniqueLocations[location] = true
	}
	//get the number of zones 
	for range uniqueLocations {
		zone++
	}

	switch zone {
	case 1:
		return 17
	case 2:
		return 24
	case 3:
		return 36
	case 4:
		return 48
	default:
		return 50

	}


}


	

	



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
