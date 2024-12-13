import { Device } from 'react-native-ble-plx';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { act } from 'react';

import { BleManager } from 'react-native-ble-plx';

interface RssiAndDetruments {
	connectForSeming: (devices: Device[], bleManager: BleManager) => Promise<Device[]>;
	streeamRssiToServer: (connectedDevices: Device[]) => Promise<void>;
}

function getRssiFromDetruments(): RssiAndDetruments {
	const connectForSeming = async (devices: Device[], bleManager: BleManager): Promise<Device[]> => {
		const verifyiesConnectedDevices: Device[] = [];
		for (const device of devices) {
			try {
				const connectedDevice = await bleManager.connectToDevice(device.id);
				await connectedDevice.discoverAllServicesAndCharacteristics();
				if (!connectedDevice) {
					console.error('Failed to connect to device:', device.name);
					continue;
				}
				verifyiesConnectedDevices.push(device);

			} catch (error) {
				console.error('Failed to get RSSI:', error);
			}

		}
		return verifyiesConnectedDevices;
	};

	const streeamRssiToServer = async (connectedDevices: Device[]): Promise<void> => {
		//	type DataRequest struct 
		//ESP1ID int64 `json:"ESP1ID"`
		//ESP2ID int64 `json:"ESP2ID"`
		//RSSI1  int64 `json:"RSSI1"`
		//RSSI2  int64 `json:"RSSI2"`
		//       TODO: retun a list fromm rssi and device names and sort them by higget rssi wuith the name

		const token = await AsyncStorage.getItem('userToken');

		const ws = new WebSocket(`ws://192.168.1.68:8082/dataingestion?token=${token}`);

		ws.onopen = () => {
			console.log('WebSocket connected');

			setInterval(async () => {
				const listOfRssisandNames = [];

				for (const device of connectedDevices) {
					try {
						const rssi = await device.readRSSI();
						listOfRssisandNames.push({ name: device.name || '', rssi: rssi.rssi || 0 });
						console.log("Device name:", device.name, "RSSI:", rssi.rssi);
					} catch (error) {
						console.error('Failed to get RSSI:', error);
					}
				}

				//filter for if the rssi is exalctly 0
				listOfRssisandNames.filter((device) => device.rssi !== 0);

				listOfRssisandNames.sort((a, b) => b.rssi - a.rssi);
				console.log("Sorted list:", listOfRssisandNames);

				if (listOfRssisandNames.length >= 2) {
					ws.send(JSON.stringify({
						//slice the nam,e to the last 4 characters
						ESP1ID: parseInt(listOfRssisandNames[0].name.slice(-4)),
						RSSI1: listOfRssisandNames[0].rssi,
						ESP2ID: parseInt(listOfRssisandNames[1].name.slice(-4)),
						RSSI2: listOfRssisandNames[1].rssi
					}));
				} else {
					console.warn("Not enough devices to send data");
				}
			}, 5000); // Repeat every 5 seconds
		};

		ws.onmessage = (e) => {
			// A message was received from the server
			console.log('Received from server:', e.data);
		};

		ws.onerror = (e) => {
			// An error occurred
			console.log('WebSocket error:', e);
		};

		ws.onclose = (e) => {
			// Connection closed
			console.log('WebSocket closed:', e.code, e.reason);
		};
	}

	return {
		connectForSeming,
		streeamRssiToServer
	};
}
export default getRssiFromDetruments;

