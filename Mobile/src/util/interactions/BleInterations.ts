import { BleManager, Device, Characteristic } from 'react-native-ble-plx';
import base64 from "react-native-base64";
import { useState } from 'react';



interface BleInteractions {
	sendToken(token: string): Promise<Device | null>;
	receiveToken: () => Promise<string>;
	readRssi: () => Promise<number>;
}

function espInteractions(bleManager: BleManager, device: Device): BleInteractions {
	const SERVICE_UUID = '4fafc201-1fb5-459e-8fcc-c5c9c331914b';
	const CHARACTERISTIC_UUID = 'beb5483e-36e1-4688-b7f5-ea07361b26a8';

	const sendToken = async (token: string): Promise<Device | null> => {
		try {
			// Connect if needed and discover all services and characteristics
			const connectedDevice = await bleManager.connectToDevice(device.id);
			await connectedDevice.discoverAllServicesAndCharacteristics();

			// Write the token to the characteristic (must be base64 encoded)
			await connectedDevice.writeCharacteristicWithResponseForService(
				SERVICE_UUID,
				CHARACTERISTIC_UUID,
				base64.encode(token)
			);

			return connectedDevice
		} catch (error) {
			console.error('Failed to send token:', error);
			return null;
		}
	};

	const receiveToken = async (): Promise<string> => {
		try {
			// Connect if needed and discover all services and characteristics
			const connectedDevice = await bleManager.connectToDevice(device.id);
			await connectedDevice.discoverAllServicesAndCharacteristics();

			// Read the characteristic value
			const characteristic: Characteristic = await connectedDevice.readCharacteristicForService(
				SERVICE_UUID,
				CHARACTERISTIC_UUID
			);

			if (characteristic && characteristic.value) {
				// Decode the base64-encoded value
				const decodedValue = base64.decode(characteristic.value);
				return decodedValue;
			}

			return '';
		} catch (error) {
			console.error('Failed to receive token:', error);
			return '';
		}
	};

	const readRssi = async (): Promise<number> => {
		try {
			const rssi = await device.readRSSI();
			console.log('RSSI:', rssi.rssi);
			if (rssi.rssi) {
				return rssi.rssi;
			}
			return 0;
		} catch (error) {
			console.error('Failed to read RSSI:', error);
			return -1;
		}

	}


	return {
		sendToken,
		receiveToken,
		readRssi,
	};
}

export default espInteractions;
