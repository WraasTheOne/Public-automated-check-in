#GetUserIDReadis
import redis

class GetUserIDReadis:
    def __init__(self):
        self.client = redis.StrictRedis(host='localhost', port=6379, db=0)

    def get_user_id(self, token):
        try:
            user_id = self.client.get(token)
            if not user_id:
                raise Exception(f"No user ID found for token {token}")
            return user_id.decode()
        except redis.RedisError as e:
            raise Exception(f"Failed to get user ID from Redis: {e}")

