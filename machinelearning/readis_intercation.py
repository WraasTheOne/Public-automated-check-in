import redis

class ReadStreamReadsOperation:
    def __init__(self, host='localhost', port=6380, db=0):
        self.client = redis.StrictRedis(host=host, port=port, db=db)
        # print connection status


    def get_keys_from_stream(self):
        keys = self.client.keys()
        return keys

    def read_stream(self, key):
        messages = self.client.xrange(key)
        return messages

    def delete_message_from_stream(self, stream_name, message_id):
        try:
            self.client.xdel(stream_name, message_id)
        except redis.RedisError as e:
            raise Exception(f"Failed to delete message from Redis stream: {e}")

    

class GetUserIDReadis:
    def __init__(self):
        self.client = redis.StrictRedis(host='localhost', port=6379, db=0)

    def get_user_id(self, token):
        try:
            user_id = self.client.get(token)
            if not user_id:
                raise Exception(f"This user ID for token {token} was not found")
            return user_id
        except Exception as e:  # Use 'except', not 'catch'
            return None

