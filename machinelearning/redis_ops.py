import redis

class RedisOperations:
    def __init__(self, host='localhost', port=6379, db=0):
        self.client = redis.StrictRedis(host=host, port=port, db=db)

    def read_first_from_stream(self, stream_name):
        try:
            messages = self.client.xread({stream_name: "0"}, count=1, block=0)
            if not messages:
                raise Exception(f"No messages found in stream {stream_name}")
            stream, message_data = messages[0]
            message_id, values = message_data[0]
            return {k.decode(): v.decode() for k, v in values.items()}, message_id
        except redis.RedisError as e:
            raise Exception(f"Failed to read from Redis stream: {e}")

    def delete_message_from_stream(self, stream_name, message_id):
        try:
            self.client.xdel(stream_name, message_id)
        except redis.RedisError as e:
            raise Exception(f"Failed to delete message from Redis stream: {e}")