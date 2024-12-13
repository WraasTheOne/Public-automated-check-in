package util

import (
	"log"
	"github.com/joho/godotenv"
	"os"
	"crypto/rand"
	"crypto/hmac"
	"crypto/sha256"
	"encoding/hex"
)


func GenerateChallenge(length int) (string, error) {
    // Create a byte slice of the desired length
    bytes := make([]byte, length)
    _, err := rand.Read(bytes)
    if err != nil {
        return "", err
    }
    return hex.EncodeToString(bytes), nil
}

func GenerateChallengeKey( token string, challenge string) string {
	// join the token and challenge
	challengeKey := token + challenge
	// hash the key 
	hash := sha256.Sum256([]byte(challengeKey))

	return hex.EncodeToString(hash[:])
}

func HashMsg(payload, key string) string {
	// Create a new HMAC by defining the hash type and the key
	mac := hmac.New(sha256.New, []byte(key))
	// Write the payload into it
	mac.Write([]byte(payload))
	// Compute the HMAC sum
	hashBytes := mac.Sum(nil)

	// Convert the byte slice to a hexadecimal string
	hash := hex.EncodeToString(hashBytes)
	return hash
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
