import socket
import asyncio
import json
from fastapi import FastAPI

app = FastAPI()

# Handle incoming TCP connections
async def handle_client(reader, writer):
    addr = writer.get_extra_info('peername')
    print(f"Received connection from {addr}")

    while True:
        data = await reader.read(100)  # Read up to 100 bytes
        if not data:
            print(f"Connection closed by {addr}")
            break

        message = data.decode()
        print(f"Received: {message}")

        try:
            # Parse the received data as JSON
            parsed_data = json.loads(message)
            print(f"Parsed JSON: {parsed_data}")
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
@app.get("/")
async def read_root():
    return {"message": "FastAPI TCP Server is running"}

# Run the TCP server alongside FastAPI
@app.on_event("startup")
async def on_startup():
    asyncio.create_task(start_tcp_server())

