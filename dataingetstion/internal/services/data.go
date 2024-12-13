package services

import (
	"encoding/json"
	"log"
	"net/http"
	"time"

	"Public-automated-check-in/dataingestion/internal/redisdb"

	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
)

type DataRequest struct {
	ESP1ID int64 `json:"ESP1ID"`
	ESP2ID int64 `json:"ESP2ID"`
	RSSI1  int64 `json:"RSSI1"`
	RSSI2  int64 `json:"RSSI2"`
}

// WebSocket Upgrader
var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool {
		return true
	},
}

// WebSocketHandler handles WebSocket connections
func WebSocketHandler(c *gin.Context) {
    // If using auth.AuthMiddleware, it should set something in context if authenticated, 
    // or have already returned a 401/403 before reaching here.

    // For example, if you're passing token as query param, you might validate it here:
    token := c.Query("token")
    if token == "" {
        // Respond with normal HTTP error if no token is provided, and return.
        c.JSON(http.StatusUnauthorized, gin.H{"error": "Authorization token is required"})
        return
    }

    // If you need additional token validation that is not handled by the middleware,
    // do it here. If invalid, return an HTTP error and do not upgrade.

    conn, err := upgrader.Upgrade(c.Writer, c.Request, nil)
    if err != nil {
        log.Printf("Failed to upgrade connection: %v", err)
        return
    }
    defer conn.Close()

    log.Println("WebSocket connection established")

    // Since the connection is upgraded, do not use c.JSON or c.Abort anymore.
    // Use conn.WriteMessage(...) or conn.ReadMessage(...) to communicate.

    // Example of a simple read/write loop:
    resetCountdown := make(chan bool)
    quit := make(chan bool)

    go func() {
        timer := time.NewTimer(30 * time.Second)
        defer timer.Stop()

        for {
            select {
            case <-resetCountdown:
                if !timer.Stop() {
                    <-timer.C
                }
                timer.Reset(30 * time.Second)
            case <-timer.C:
                log.Println("No data received for 30 seconds, closing connection")
                conn.WriteMessage(websocket.TextMessage, []byte("Connection closing due to inactivity"))
                quit <- true
                return
            }
        }
    }()

    for {
        select {
        case <-quit:
            log.Println("WebSocket handler shutting down")
            return
        default:
            _, message, err := conn.ReadMessage()
            if err != nil {
                log.Printf("Error reading message: %v", err)
                return
            }

            // Reset the countdown timer
            resetCountdown <- true

            var dataReq DataRequest
            if err := json.Unmarshal(message, &dataReq); err != nil {
                log.Printf("Invalid message format: %v", err)
                conn.WriteMessage(websocket.TextMessage, []byte("Invalid data format"))
                continue
            }

            // Validate the data request
            if dataReq.ESP1ID == 0 || dataReq.ESP2ID == 0 || dataReq.RSSI1 == 0 || dataReq.RSSI2 == 0 {
                log.Printf("Invalid data: %v", dataReq)
                conn.WriteMessage(websocket.TextMessage, []byte("Invalid data"))
                continue
            }

            // Add to Redis (assuming redisdb.AddToStream works as intended)

            if err := redisdb.InitRedis(); err != nil {
                log.Printf("Failed to initialize Redis: %v", err)
                conn.WriteMessage(websocket.TextMessage, []byte("Failed to ingest data"))
                continue
            }
            defer redisdb.CloseRedis()


			//convert eact 
            data := map[string]interface{}{
                "esp1id": dataReq.ESP1ID,
                "rssi1":  dataReq.RSSI1,
                "esp2id": dataReq.ESP2ID,
                "rssi2":  dataReq.RSSI2,
            }

            if err := redisdb.AddToStream(token, data); err != nil {
                log.Printf("Failed to add data to Redis stream: %v", err)
                conn.WriteMessage(websocket.TextMessage, []byte("Failed to ingest data"))
                continue
            }

            log.Printf("Data ingested: %v", data)
            conn.WriteMessage(websocket.TextMessage, []byte("Data ingested successfully"))
        }
    }
}

