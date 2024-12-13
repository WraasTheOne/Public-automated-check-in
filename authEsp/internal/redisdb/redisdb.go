package redisdb

import (
	"context"
	"fmt"
	"time" // Import the time package for TTL functionality

	"github.com/go-redis/redis/v8" // Use the correct Redis package for context support
)

var rdb *redis.Client
var ctx = context.Background() // Global context

// InitRedis initializes the Redis client
func InitRedis() error {
	rdisClient := redis.NewClient(&redis.Options{
		Addr:     "localhost:6381",
		Password: "", // No password set
		DB:       0,  // Use default DB
	})

	_, err := rdisClient.Ping(ctx).Result()
	if err != nil {
		return fmt.Errorf("failed to connect to Redis: %v", err)
	}
	rdb = rdisClient
	return nil
}

// CloseRedis closes the Redis client connection
func CloseRedis() {
	rdb.Close()
}

func StoreChallengeForEsp(challengeKey string, challenge string) error {
	err := rdb.Set(ctx, challengeKey, challenge, 2*time.Minute).Err()
	if err != nil {
		return fmt.Errorf("failed to store challenge: %v", err)
	}

	return nil
}

func MatchChallengeForEsp(challengeKey string) (string, error) {
	challenge, err := rdb.Get(ctx, challengeKey).Result()
	if err == redis.Nil {
		return "", fmt.Errorf("challenge not found")
	} else if err != nil {
		return "", fmt.Errorf("failed to get challenge: %v", err)
	}

	err = rdb.Del(ctx, challengeKey).Err()
	if err != nil {
		return "", fmt.Errorf("failed to delete challenge: %v", err)
	}

	return challenge, nil
}






