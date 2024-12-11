package services

import (
	"encoding/json"
	"log"
	"net/http"

	"Public-automated-check-in/dataingestion/internal/redisdb"

	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
)

type DataRequest struct {
	ID        string `json:"id"`
	Payload   string `json:"payload"`
	Timestamp int64  `json:"timestamp"`
}

// WebSocket Upgrader
var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool {
		return true
	},
}

// WebSocketHandler handles WebSocket connections
func WebSocketHandler(c *gin.Context) {
	conn, err := upgrader.Upgrade(c.Writer, c.Request, nil)
	if err != nil {
		log.Printf("Failed to upgrade connection: %v", err)
		return
	}
	defer conn.Close()

	log.Println("WebSocket connection established")

	for {
		// Read data from the WebSocket client
		_, message, err := conn.ReadMessage()
		if err != nil {
			log.Printf("Error reading message: %v", err)
			break
		}

		var dataReq DataRequest
		err = json.Unmarshal(message, &dataReq)
		if err != nil {
			log.Printf("Invalid message format: %v", err)
			conn.WriteMessage(websocket.TextMessage, []byte("Invalid data format"))
			continue
		}

		// Prepare data for Redis stream
		data := map[string]interface{}{
			"id":        dataReq.ID,
			"payload":   dataReq.Payload,
			"timestamp": dataReq.Timestamp,
		}

		// Add data to Redis stream
		err = redisdb.AddToStream("data_stream", data)
		if err != nil {
			log.Printf("Failed to add data to Redis stream: %v", err)
			conn.WriteMessage(websocket.TextMessage, []byte("Failed to ingest data"))
			continue
		}

		log.Printf("Data ingested: ID=%s, Payload=%s", dataReq.ID, dataReq.Payload)
		conn.WriteMessage(websocket.TextMessage, []byte("Data ingested successfully"))
	}
}
