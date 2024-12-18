import time
from db import MySQLDatabase

from readis_intercation import ReadStreamReadsOperation
from readis_intercation import GetUserIDReadis

from ml_model import MachineLearningModel


class MachineLearningService:
    def __init__(self):
        self.db = MySQLDatabase(user="root", password="rootpassword", host="localhost", port="3307", database="userdb")
        self.read_stream = ReadStreamReadsOperation()
        self.get_user_id = GetUserIDReadis()
        self.ml_model = MachineLearningModel()
        self.ml_model.load("model.pkl") 

    def process_stream(self):
        while True:
            keys = self.read_stream.get_keys_from_stream()
            for key in keys:
                user_id = self.get_user_id.get_user_id(key.decode("utf-8"))
                if user_id is None:
                    continue
                data = self.read_stream.read_stream(key)

                check_in_startus = self.db.get_checkin_status(user_id)
                for entry in data:

                    date = entry[0].decode("utf-8")
                    rssi1 = entry[1][b'rssi1'].decode("utf-8")
                    esp1id = entry[1][b'esp1id'].decode("utf-8")
                    rssi2 = entry[1][b'rssi2'].decode("utf-8")
                    esp2id = entry[1][b'esp2id'].decode("utf-8")

                    #preint all data:
                    get_current_location = self.db.get_current_location(esp1id)
                    print(get_current_location[0])

                    prediction = self.ml_model.predict(esp1id, rssi1, esp2id, rssi2, 1)

                    print("hour: ", time.strftime("%H:%M:%S", time.localtime()))
                    print("are we in transport? ", prediction)

                    if prediction == "no" and self.db.get_is_in_transport(user_id) == 0:
                        print("not in transport and alrdy checked out")
                        self.read_stream.delete_message_from_stream(key, date)
                        continue


                    if prediction == "yes":
                        self.db.set_transport_status(user_id, 1)
                    else:
                        self.db.set_transport_status(user_id, 0)
                        self.read_stream.delete_message_from_stream(key, date)
                        print("not in transport")
                        continue
                    if prediction == "yes" and check_in_startus == 0:
                        if self.db.get_checkin_status(user_id) == 1:
                            continue
                        self.db.update_checkin_status(user_id, 1)
                        print("---------checkin---------")
                        self.db.register_trip(user_id, get_current_location[0])

                    self.db.register_trip_data(get_current_location[0])

                    self.read_stream.delete_message_from_stream(key, date)
            time.sleep(1)

if __name__ == "__main__":
    service = MachineLearningService()
    service.process_stream()
