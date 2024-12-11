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
	ESP1ID int64  `json:"ESP1ID"`
	ESP2ID int64  `json:"ESP2ID"`
	RSSI1  string `json:"RSSI1"`
	RSSI2  string `json:"RSSI2"`
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

	token := c.GetHeader("Authorization")
	if token == "" {
		c.JSON(http.StatusUnauthorized, gin.H{"error": "Authorization token is required"})
		c.Abort()
		return
	}

	err = redisdb.InitRedis()
	if err != nil {
		log.Fatalf("Failed to initialize Redis: %v", err)
	}
	defer redisdb.CloseRedis()

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
			"esp1id": dataReq.ESP1ID,
			"rssi1":  dataReq.RSSI1,
			"esp2id": dataReq.ESP1ID,
			"rssi2":  dataReq.RSSI1,
		}

		if dataReq.ESP1ID == 0 || dataReq.ESP2ID == 0 || dataReq.RSSI1 == "" || dataReq.RSSI2 == "" {
			log.Printf("Invalid data: %v", dataReq)
			conn.WriteMessage(websocket.TextMessage, []byte("Invalid data"))
			continue
		}

		// Add data to Redis stream
		err = redisdb.AddToStream(token, data)
		if err != nil {
			log.Printf("Failed to add data to Redis stream: %v", err)
			conn.WriteMessage(websocket.TextMessage, []byte("Failed to ingest data"))
			continue
		}

		log.Printf("Data ingested: %v", data)
		conn.WriteMessage(websocket.TextMessage, []byte("Data ingested successfully"))
	}
}
