package util

import (
	"log"
	"os"
	"github.com/joho/godotenv"
	"fmt"
)


var zones = map[int][]string{
		1: { "Sluseholmen St.", "Nørreport St.", "København H", "Østerport St.", "Amagerbro St.", "Christianshavn St.", },
		2: { "Valby St.", "Frederiksberg St.", "Nordhavn St.",  "Hellerup St.", },
		3: { "Rødovre St.",  "Glostrup St.", "Albertslund St.", "IshøjSt.", },
		4: { "Høje Taastrup St.", "Greve St.",  "Ballerup St.", "Farum St.", "Kokkedal St.", },
	}
// get how different the zones are from the locations thre can be multiple zones
func GetZones(locations []string) int {
	// get unique locations  from the locations 
	//count the number of diffent zones the user has been we can use location to 

	uniquelocations := make(map[string]bool)
	for _, location := range locations {
		uniquelocations[location] = true
	}
	//map[Sluseholmen St.:true]

	// get the zones for the locations return the number of zones 
	// the user has been tAken from the locations
	zone := 0
	for key, value := range zones {
		for _, location := range value {
			if _, ok := uniquelocations[location]; ok {
				zone = key
				break
			}
		}
	}
	fmt.Println(zone)
	return zone

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

func GetPrice(zones int) int {
	//switch statement to get the price of the zones

	switch zones {
	case 1:
		return 17
	case 2:
		return 24
	case 3:
		return 36
	case 4:
		return 48
	default:
		return 0

	}

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
