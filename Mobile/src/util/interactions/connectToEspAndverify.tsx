import { BleManager, Device } from "react-native-ble-plx";
import serverInterations from "./serverInterations";
import espInteractions from "./BleInterations";

//its a lsit of devices
export const connectToEspAndverify = async (bleManager: BleManager, devices: Device[]): Promise<Device[] | null> => {
	try {
		const connections = devices.map(async (device): Promise<Device | null> => {
			try {
				console.log("Connecting to device:", device.id);

				const espInteract = espInteractions(bleManager, device); // Replace with your implementation
				const serverToken = await serverInterations().getToken(); // Replace with your implementation
				const connectedDevice = await espInteract.sendToken(serverToken);
				const token = await espInteract.receiveToken();
				console.log("Received token:", token);
				//send the token to the server
				const sendToken = await serverInterations().sendResponeToken(token);
				if (sendToken) {
					return connectedDevice;
				}
				return null;
			} catch (error) {
				console.error("Error connecting to device:", error);
				device.cancelConnection();
				return null;
			}
		}
		);
		const connectedDevices = await Promise.all(connections);

		return connectedDevices.filter((dev) => dev !== null);
	} catch (error) {
		console.error("Error in BLE scan and connect:", error);
		return [];
	}

}
