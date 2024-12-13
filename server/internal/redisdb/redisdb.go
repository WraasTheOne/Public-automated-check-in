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

// SetWithExpiration sets a key-value pair in Redis with an expiration time
func SetWithExpiration(key string, value interface{}, expiration int64) error {
	if redisClient == nil {
		return fmt.Errorf("Redis client is not initialized")
	}

	err := redisClient.Set(key, value, time.Duration(expiration)*time.Second).Err()
	fmt.Println("the session is ", key)
	if err != nil {
		return fmt.Errorf("failed to set key in Redis: %v", err)
	}

	return nil
}

// CloseRedis closes the Redis client connection
func CloseRedis() {
	fmt.Println("Closing Redis connection")
	if redisClient != nil {
		redisClient.Close()
	}
}
