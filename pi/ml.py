import json
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.linear_model import LogisticRegression

#the data: the rssi data from the 3 access points and the inside/outside status
#    {"alpha": -72, "beta": -70, "omega": -49, "inside the station": true},
#    {"alpha": -71, "beta": -82, "omega": -53, "inside the station": true},
#    {"alpha": -82, "beta": -76, "omega": -57, "inside the station": false},

## Load data from logging.json
with open("C:\\Users\\alexa\\Sander_Suff\\p5\\Public-automated-check-in\\pi\\loging.json", "r") as file:
    data = json.load(file)

#load data into a pandas dataframe
df = pd.DataFrame(data)

#split the data into features and target
X = df[['alpha', 'beta', 'omega']]
y = df['inside the station']

#split the data into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
#standardize the data
scaler = StandardScaler()
#fit the scaler on the training data and transform the training data
X_train = scaler.fit_transform(X_train)
X_test = scaler.transform(X_test)

#train the model
model = LogisticRegression()
model.fit(X_train, y_train)

#use the alpha and omega to draw a LogisticRegression plot set the color to blue and red based on the inside/outside status

def predict_inside_station(alpha, beta, omega):
    # Prepare the input data as a DataFrame with correct column names
    input_data = pd.DataFrame([[alpha, beta, omega]], columns=["alpha", "beta", "omega"])
    
    # Scale the input data
    input_data_scaled = scaler.transform(input_data)
    
    # Make prediction
    prediction = model.predict(input_data_scaled)
    
    # Return result as True/False
    return bool(prediction[0])


