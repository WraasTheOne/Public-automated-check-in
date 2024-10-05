from flask import Flask, Response, render_template, request, redirect, url_for
import requests
import json
import time
import sys
import ml


#open a file "loging.json" to write in the bottom of the file




app = Flask(__name__)

# FastAPI endpoint to get RSSI data
FASTAPI_URL = "http://192.168.1.68:8000/rssis"

def get_one_fetch():
    response = requests.get(FASTAPI_URL)
    if response.status_code == 200:
        rssi_data = response.json()  # Get the RSSI data
        return rssi_data
    else:
        # Return a fallback dictionary when the API fails
        return {'alpha': 'No Data', 'beta': 'No Data', 'omega': 'No Data'}


def fetch_rssi_data():
    while True:
        try:
            response = requests.get(FASTAPI_URL)
            if response.status_code == 200:
                rssi_data = response.json()  # Get the RSSI data
                # Predict the location based on the RSSI data
                prediction = ml.predict_inside_station(rssi_data['alpha'], rssi_data['beta'], rssi_data['omega'])
                if prediction == True:
                    rssi_data = {'alpha': rssi_data['alpha'], 'beta': rssi_data['beta'], 'omega': rssi_data['omega'], 'status': True}
                else:
                    rssi_data = {'alpha': rssi_data['alpha'], 'beta': rssi_data['beta'], 'omega': rssi_data['omega'], 'status': False}
                json_data = json.dumps(rssi_data)  # Convert to JSON string
                yield f"data: {json_data}\n\n"  # Properly format for SSE
            else:
                yield f"data: {{'alpha': 'No Data', 'beta': 'No Data', 'omega': 'No Data'}}\n\n"
        except Exception as e:
            yield f"data: {{'error': 'Failed to retrieve RSSI data: {e}'}}\n\n"
        
        time.sleep(2)  # Wait for 2 seconds before fetching the data again

@app.route('/')
def index():
    return render_template('index.html')

# submit app
#        <input type="checkbox" name="checkInd">Indside/Outside - On/Off</input>
@app.route('/submit', methods=['POST'])
def save_info():
    rssis = get_one_fetch()
    # Get the checkbox value
    checkInd = request.form.get('checkInd')

    if checkInd == "inside":
        checkInd = True
    else:
        checkInd = False

    rssi_data = {
        'alpha': rssis['alpha'],
        'beta': rssis['beta'],
        'omega': rssis['omega'],
        'inside the station': checkInd,
    }
    loging(rssi_data)

    # Redirect to the root URL
    return redirect(url_for('index'))


#loging.json add the datra in the bottom of the file as json
def loging(data):
    with open('loging.json', 'a') as f:
        json.dump(data, f)
        f.write('\n')
        #close the file
        f.close()


@app.route('/stream')
def stream():
    return Response(fetch_rssi_data(), mimetype='text/event-stream')

if __name__ == '__main__':
    app.run(debug=True, threaded=True, host='192.168.1.68', port=5000)

