import { BleManager, Device } from "react-native-ble-plx";
import AsyncStorage from "@react-native-async-storage/async-storage";

import serverInterations from "./interactions/serverInterations";
import espInteractions from "./interactions/BleInterations";


// Initialize BLE Manager
const bleManager = new BleManager();

// Function to start BLE scanning and connect to devices
const discoveredDevices: Device[] = [];
const verifiedDevices: Device[] = [];


const checkIfstillConnected = async (device: Device): Promise<boolean> => {
	try {
		const isConnected = await device.isConnected();
		return isConnected;
	} catch (error) {
		console.error("Error checking connection status:", error);
		return false;
	}
}


export const startBleScanAndConnect = async (): Promise<Device[]> => {
	try {
		if (verifiedDevices.length > 0) {
			const mappedDevices = verifiedDevices.map(async (device) => {
				const isConnected = await checkIfstillConnected(device);
				if (!isConnected) {
					console.log("Device disconnected:", device.id);
					return null;
				}
				if (verifiedDevices.includes(device)) {
					return null;
				}
				return device;
			});
			if (mappedDevices.length > 0) {
				const connectedDevices = await Promise.all(mappedDevices);
				verifiedDevices.push(...connectedDevices.filter((dev) => dev !== null));
				return verifiedDevices;
			}
		}
		console.log("Starting BLE scan...");

		let coutOfDevices = 0;
		await new Promise<void>((resolve) => {
			bleManager.startDeviceScan(null, null, (error, device) => {
				if (error) {
					console.error("Error during scan:", error);
					bleManager.stopDeviceScan();
					resolve();
					return;
				}
				if (device) {
					console.log("we found a device", device.name);
					coutOfDevices++;
				}
				if (coutOfDevices > 10) {
					bleManager.stopDeviceScan();
					resolve();
				}
				// Filter devices with names starting with "ESP32"
				if (device?.name && device.name.startsWith("ESP32")) {
					if (!discoveredDevices.some((d) => d.id === device.id)) {
						discoveredDevices.push(device);
						console.log("Discovered device:", device.name, device.id);
					}
				}
			});
		});

		console.log("Discovered devices:", discoveredDevices);

		const connectedDevices = await Promise.all(
			discoveredDevices.map(async (device): Promise<Device | null> => {
				try {
					console.log("Connecting to device:", device.id);

					// Interact with ESP32
					const espInteract = espInteractions(bleManager, device); // Replace with your implementation
					const serverToken = await serverInterations().getToken(); // Replace with your implementation
					const connectedDevice = await espInteract.sendToken(serverToken);
					const token = await espInteract.receiveToken();

					//TODO:              to see if the token is valid :) 😆

					console.log("Token received:", token);

					// Return connected device if successful
					return connectedDevice;
				} catch (error) {
					console.error("Error connecting to device:", error);
					return null;
				}
			})
		);

		// Filter verified devices
		//const verifiedDevices = connectedDevices.filter(
		//	(dev): dev is Device => dev !== null
		//);
		verifiedDevices.push(...connectedDevices.filter((dev) => dev !== null));
		console.log("Verified devices:", verifiedDevices);

		// Save verified devices to AsyncStorage
		await AsyncStorage.setItem("verifiedDevices", JSON.stringify(verifiedDevices));

		return verifiedDevices;
	} catch (error) {
		console.error("Error in BLE scan and connect:", error);
		return [];
	}
};

