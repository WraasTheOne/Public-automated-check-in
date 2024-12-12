import grpc
from concurrent import futures
import machinelearning_pb2
import machinelearning_pb2_grpc
from db import MySQLDatabase
from redis_ops import RedisOperations
from ml_model import MachineLearningModel
import calc_pb2
import calc_pb2_grpc

class MachineLearningService(machinelearning_pb2_grpc.MachineLearningServiceServicer):
    def __init__(self):
        self.db = MySQLDatabase(user="root", password="rootpassword", host="127.0.0.1", port="3307", database="userdb")
        self.redis = RedisOperations()
        self.ml_model = MachineLearningModel()
        self.ml_model.load("model.pkl")
    
    def call_calc_trip(tripid):
    # Connect to the gRPC server
        with grpc.insecure_channel('localhost:50051') as channel:
            # Create a stub (client) for the AuthService
            stub = calc_pb2_grpc.AuthServiceStub(channel)

            # Create a CalcTripRequest message
            request = calc_pb2.CalcTripRequest(tripid=tripid)

            # Call the CalcTrip RPC
            try:
                response = stub.CalcTrip(request)
                print(f"Status: {response.status}")
                print(f"Message: {response.message}")
            except grpc.RpcError as e:
                print(f"gRPC call failed: {e.details()} (Code: {e.code()})")

    def Predict(self, request, context):
        user_id = request.userID
        token = request.token

        try:
            # Step 1: Register the trip
            trip_id = self.db.register_trip(user_id)
            print(f"Registered trip with ID: {trip_id}")

            # Step 2: Process Redis stream
            timeout = 30  # Timeout in seconds
            while timeout > 0:
                data, message_id = self.redis.read_first_from_stream(token)

                # Step 3: Extract and process data
                esp1_id = int(data['ESP1ID'])
                location, passengers = self.db.get_current_location(esp1_id)
                print(f"Location: {location}, Passengers: {passengers}")
                
                if self.ml_model.predict(data['RSSI1'], data['RSSI2'], passengers) == "no":
                    print("Prediction: No passengers detected")
                    continue
                
                

                # Step 4: Store trip data
                self.db.register_trip_data(trip_id, location)
                print(f"Stored trip data for trip ID {trip_id} at location {location}")

                # Step 5: Delete message from Redis
                self.redis.delete_message_from_stream(token, message_id)
                print(f"Deleted message ID {message_id} from Redis stream")

                timeout -= 1  # Decrement timeout for each successful operation

            return machinelearning_pb2.PredictResponse(
                serviceStatus=True,
                message="Prediction process completed successfully"
            )
        except Exception as e:
            print(f"Error in Predict: {e}")
            return machinelearning_pb2.PredictResponse(
                serviceStatus=False,
                message=str(e)
            )
        finally:
            # Ensure resources are cleaned up
            self.call_calc_trip(trip_id)
            self.db.close()
            self.redis.close()
        
            

def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    machinelearning_pb2_grpc.add_MachineLearningServiceServicer_to_server(MachineLearningService(), server)
    server.add_insecure_port('[::]:50051')
    server.start()
    print("Server started on port 50051")
    server.wait_for_termination()

if __name__ == "__main__":
    serve()