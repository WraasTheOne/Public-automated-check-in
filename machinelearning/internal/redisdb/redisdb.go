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
func ReadFirstFromStream(streamName string) (map[string]string, string, error) {
	// Read one message from the stream
	messages, err := rdb.XRead(&redis.XReadArgs{
		Streams: []string{streamName, "0"}, // Start reading from the beginning
		Count:   1,                         // Read only the first message
		Block:   0,                         // Block until a message is available
	}).Result()
	if err != nil {
		return nil, "", fmt.Errorf("failed to read data from Redis stream: %v", err)
	}

	// Extract the first message
	if len(messages) == 0 || len(messages[0].Messages) == 0 {
		return nil, "", fmt.Errorf("no messages found in stream: %s", streamName)
	}
	message := messages[0].Messages[0]

	values := make(map[string]string)
	for k, v := range message.Values {
		if str, ok := v.(string); ok {
			values[k] = str
		} else {
			return nil, "", fmt.Errorf("non-string value found in message: %v", v)
		}
	}
	return values, message.ID, nil
}

func DeleteMessageFromStream(streamName string, messageID string) error {
	_, err := rdb.XDel(streamName, messageID).Result()
	if err != nil {
		return fmt.Errorf("failed to delete message from Redis stream: %v", err)
	}
	return nil
}
