import { Device } from 'react-native-ble-plx';
import AsyncStorage from '@react-native-async-storage/async-storage';

export const getRssiFromDevises = async (devices: Device[]): Promise<void> => {
	const readRssi = async (device: Device): Promise<number> => {
		try {
			const rssi = await device.readRSSI();
			console.log("Device:", device.name, "RSSI:", rssi.rssi);
			if (rssi.rssi) {
				return rssi.rssi;
			}
			return 9999;
		} catch (error) {
			console.error("Failed to read RSSI for device:", device.name, error);
			return 9999;
		}
	}

	const rssiValues = await Promise.all(devices.map(async (device) => {
		return await readRssi(device);
	}));
	console.log("RSSI values:", rssiValues);

	/// TODO:       send the rssi values to the server
	if (rssiValues.length > 0) {
		// set stroage a truu value
		await AsyncStorage.setItem("BackIsCheakdIn", "true");
	}


}



