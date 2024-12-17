import { Device } from 'react-native-ble-plx';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { act } from 'react';

import { BleManager } from 'react-native-ble-plx';

interface RssiAndDetruments {
	connectForSeming: (devices: Device[], bleManager: BleManager) => Promise<Device[]>;
	streeamRssiToServer: (connectedDevices: Device[]) => Promise<Boolean>;
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
				console.log('error to coonect', error)
			}

		}
		return verifyiesConnectedDevices;
	};

	const streeamRssiToServer = async (connectedDevices: Device[]): Promise<boolean> => {
		let streamedData = false;
		const token = await AsyncStorage.getItem('userToken');
		const ws = new WebSocket(`ws://192.168.102.10:8082/dataingestion?token=${token}`);

		const DELAY_BETWEEN_SENDS = 3000;

		return new Promise<boolean>((resolve, reject) => {
			ws.onopen = async () => {
				console.log('WebSocket connected');

				try {
					for (let i = 0; i < 5; i++) {
						const devicesRssi = [];

						for (const device of connectedDevices) {
							try {
								const rssi = await device.readRSSI();
								if (rssi?.rssi && rssi.rssi !== 0) {
									devicesRssi.push({ name: device.name ?? '', rssi: rssi.rssi });
									console.log("Device name:", device.name, "RSSI:", rssi.rssi);
								}
							} catch (error) {
								console.error('Failed to get RSSI:', error);
							}
						}

						// Sort descending by RSSI
						devicesRssi.sort((a, b) => b.rssi - a.rssi);

						// Ensure we have at least two devices with valid RSSI
						if (devicesRssi.length >= 2) {
							const parseId = (name: string) => {
								const slicedName = name.length >= 4 ? name.slice(-4) : name;
								return parseInt(slicedName, 10) || 0;
							};

							const dataToSend = {
								ESP1ID: parseId(devicesRssi[0].name),
								RSSI1: devicesRssi[0].rssi,
								ESP2ID: parseId(devicesRssi[1].name),
								RSSI2: devicesRssi[1].rssi
							};

							ws.send(JSON.stringify(dataToSend));
							streamedData = true;

						} else {

                            break;

						}

						// Wait before sending the next packet, except after the last iteration
						if (i < 5) {
							await new Promise((res) => setTimeout(res, DELAY_BETWEEN_SENDS));
						}
					}

					// After sending 5 times, close the connection
					ws.close();
				} catch (err) {
					console.error('Error during streaming:', err);
					ws.close();
					reject(err);
				}
			};

			ws.onerror = (e) => {
				console.log('WebSocket error:', e);
				reject(e);
			};

			ws.onclose = (e) => {
				console.log('WebSocket closed:', e.code, e.reason);
				resolve(streamedData);
			};
		});
	};
	return {
		connectForSeming,
		streeamRssiToServer
	};
}
export default getRssiFromDetruments;

