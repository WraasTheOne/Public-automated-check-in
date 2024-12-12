package redisdb

import (
	"fmt"

	"github.com/go-redis/redis"
)

var rdb *redis.Client

// InitRedis initializes the Redis client
func InitRedis() error {
	rdb = redis.NewClient(&redis.Options{
		Addr: "localhost:6380", // Update if Redis is not on localhost
	})
	_, err := rdb.Ping().Result()
	return err
}

// CloseRedis closes the Redis connection
func CloseRedis() {
	rdb.Close()
}

// AddToStream adds a message to the Redis stream
func AddToStream(streamName string, data map[string]interface{}) error {
	err := rdb.XAdd(&redis.XAddArgs{
		Stream: streamName,
		Values: data,
	}).Err()
	if err != nil {
		return fmt.Errorf("failed to add data to Redis stream: %v", err)
	}
	return nil
}
