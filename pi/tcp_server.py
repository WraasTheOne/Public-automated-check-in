import socket
import asyncio
import json
from fastapi import FastAPI

app = FastAPI()

#rember we have 3 esp32: alpha, beta, omega
global alpha
global beta 
global omega
alpha = 0
beta = 0
omega = 0

#on the esp32:         snprintf(sendBuffer, sizeof(sendBuffer), "{\"device\":\"alpha\", \"value\": %d}", value);
# Handle incoming TCP connections

async def handle_client(reader, writer):
    global alpha, beta, omega
    addr = writer.get_extra_info('peername')
    print(f"Received connection from {addr}")

    while True:
        data = await reader.read(100)  # Read up to 100 bytes
        if not data:
            print(f"Connection closed by {addr}")
            break

        message = data.decode()
        try:
            # Parse the received data as JSON
            parsed_data = json.loads(message)
            #get the device name
            device = parsed_data["device"]
            #get the value
            value = parsed_data["value"]
            if device == "alpha":
                alpha = value
            elif device == "beta":
                beta = value
            elif device == "omega":
                omega = value
            print(f"Received data from {device}: the rssi is: {value}")

        except json.JSONDecodeError:
            print("Received data is not valid JSON.")

    writer.close()
    await writer.wait_closed()


# Start the TCP server
async def start_tcp_server():
    server = await asyncio.start_server(handle_client, '0.0.0.0', 12345)
    async with server:
        await server.serve_forever()

# FastAPI endpoint (optional)
@app.get("/rssis")
async def read_root():
    global alpha, beta, omega
    return {"alpha": alpha, "beta": beta, "omega": omega}

# Run the TCP server alongside FastAPI
@app.on_event("startup")
async def on_startup():
    asyncio.create_task(start_tcp_server())

