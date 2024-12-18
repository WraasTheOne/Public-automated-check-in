import joblib
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score

class MachineLearningModel:
    def __init__(self):
        self.model = None

    def train(self, csv_path, model_path):
        data = pd.read_csv(csv_path)
        #esp1id,RSSI1,esp2id,RSSI2,inOrNot,Passangercount
        X = data[["esp1id", "RSSI1", "esp2id", "RSSI2", "Passangercount"]]
        y = data['inOrNot'].map({'yes': 1, 'no': 0})

        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3, random_state=42)

        self.model = RandomForestClassifier(n_estimators=100, random_state=42)
        self.model.fit(X_train, y_train)

        y_pred = self.model.predict(X_test)
        accuracy = accuracy_score(y_test, y_pred)
        print(f"Model accuracy: {accuracy * 100:.2f}%")

        joblib.dump(self.model, model_path)

    def load(self, model_path):
        self.model = joblib.load(model_path)

    def predict(self, esp1id, rssi1, esp2id, rssi2, passenger_count):
        if self.model is None:
            raise ValueError("Model is not loaded")
        #input_data =  pd.DataFrame([[rssi1, rssi2, passenger_count]], columns=['RSSI1', 'RSSI2', 'Passangercount'])
        input_data =  pd.DataFrame([[esp1id, rssi1, esp2id, rssi2, passenger_count]], columns=['esp1id', 'RSSI1', 'esp2id', 'RSSI2', 'Passangercount'])
        prediction = self.model.predict(input_data)
        return "yes" if prediction[0] == 1 else "no"

if __name__ == "__main__":
    model = MachineLearningModel()
    model.train("train_data.csv", "model.pkl")


