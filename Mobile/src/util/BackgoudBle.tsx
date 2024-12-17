import { BleManager, Device } from "react-native-ble-plx";
import getRssiFromDetruments from "./BleRssi";
import { connectToEspAndverify } from "./interactions/connectToEspAndverify";

// Initialize BLE Manager
const bleManager = new BleManager();

// Function to start BLE scanning and connect to devices
const discoveredDevices: Device[] = [];



export const startBleScanAndConnect = async (): Promise<Boolean> => {
	try {
		const { connectForSeming, streamOnetime } = getRssiFromDetruments();
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
				if (coutOfDevices > 20) {
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


		const verifiedDevices = await connectToEspAndverify(bleManager, discoveredDevices);

		if (verifiedDevices === null) {
			return false;
		}

		console.log("Verified devices:", verifiedDevices.length);
		verifiedDevices.push(...verifiedDevices.filter((dev) => dev !== null));


		// Connect to devices for streaming RSSI
		const connectedDevices = await connectForSeming(verifiedDevices, bleManager);

		if (connectedDevices.length === 0) {
			console.error("Failed to connect to any devices");
			return false;
		}

		// Stream RSSI data to server
		const streamedData = await streamOnetime(connectedDevices);
		for (const device of connectedDevices) {
			await bleManager.cancelDeviceConnection(device.id);
		}

		return streamedData;
	} catch (error) {
		console.error("Error in BLE scan and connect:", error);
		return false;
	}
};
