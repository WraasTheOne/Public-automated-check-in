package redisdb

import (
	"fmt"
	"time"

	"github.com/go-redis/redis"
)

var redisClient *redis.Client

// InitRedis initializes the Redis client
func InitRedis() error {
	redisClient = redis.NewClient(&redis.Options{
		Addr:     "localhost:6379", // Redis address, ensure it matches your configuration
		Password: "",               // No password set (update if your Redis instance uses a password)
		DB:       0,                // Use the default DB
	})

	// Test the connection
	_, err := redisClient.Ping().Result()
	if err != nil {
		return fmt.Errorf("failed to connect to Redis: %v", err)
	}

	fmt.Println("Connected to Redis successfully")
	return nil
}

// check if token exists in Redis and update ttl to 3600 seconds
func CheckToken(token string) bool {
	_, err := redisClient.Get(token).Result()
	if err != nil {
		return false
	}
	redisClient.Expire(token, 3600*time.Second)
	return true
}

// CloseRedis closes the Redis client connection
func CloseRedis() {
	fmt.Println("Closing Redis connection")
	if redisClient != nil {
		redisClient.Close()
	}
}
