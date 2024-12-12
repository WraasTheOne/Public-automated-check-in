import { Device } from 'react-native-ble-plx';
import AsyncStorage from '@react-native-async-storage/async-storage';

export const getRssiFromDevises = async (devices: Device[]): Promise<void> => {
	console.log("Getting RSSI values for devices:", devices.map((device) => device.name));

	const readRssi = async (device: Device): Promise<Device | null> => {
		try {
			const rssi = await device.readRSSI();
			console.log("Device:", device.name, "RSSI:", rssi.rssi);
			if (rssi.rssi) {
				return rssi
			}
			return null;
		} catch (error) {
			console.error("Failed to read RSSI for device:", device.name, error);
			return null;
		}
	}


	const rssiValues = await Promise.all(devices.map(async (device) => {
		return await readRssi(device);
	}))

	const filteredRssiValues = rssiValues.filter((rssi) => rssi !== null)

	//sort the devices by RSSI highest to lowest
	filteredRssiValues.sort((a, b) => {
		if (a && b) {
			return b.rssi - a.rssi; //i have chked the rssi values
		}
		return 0;
	})

	console.log("Filtered RSSI names:", filteredRssiValues[0]?.name, filteredRssiValues[1]?.name);
	console.log("Filtered RSSI values:", filteredRssiValues[0]?.rssi, filteredRssiValues[1]?.rssi);
	const token = await AsyncStorage.getItem('userToken');
	if (token) {
		const response = await fetch('http://192.168.1.68:8001/dataingestion', {
			method: 'POST',
			headers: {
				'Content-Type': 'application/json',
				'Authorization': `Bearer ${token}`
			},
			body: JSON.stringify({
				//the last 4 of the name
				ESP1ID: filteredRssiValues[0]?.name?.slice(-4),
				ESP2ID: filteredRssiValues[1]?.name?.slice(-4),
				RSSI1: filteredRssiValues[0]?.rssi,
				RSSI2: filteredRssiValues[1]?.rssi
			})
		});
		const data = await response.json();
		if (response.ok) {
			console.log("Data sent successfully:", data);
		} else {
			console.error("Failed to send data:", data.message);
		}

	}
}
