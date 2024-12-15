import { BleManager, Device } from "react-native-ble-plx";

import espInteractions from "./BleInterations";
import server from "./serverInterations";



//its a lsit of devices
export const connectToEspAndverify = async (bleManager: BleManager, devices: Device[]): Promise<Device[] | null> => {

	const { readChallange, ConnetAndSendChallange } = espInteractions();
	const { sendChanglheToserver, GetChallengeFromServer } = server();

	const connectedDevices: Device[] = [];

	for (const device of devices) {
		try {
			if (!device.name) {
				return null;
			}
			console.log(`Connecting to device: ${device.name}`);
			const serverChallenge = await GetChallengeFromServer(device.name);
			if (!serverChallenge) {
				continue;
			}

			const connectedDevice = await ConnetAndSendChallange(serverChallenge, bleManager, device);

			if (!connectedDevice) {
				console.error(`Failed to connect to device: ${device.name}`);
				continue;
			}
			const computedChallenge = await readChallange(connectedDevice);
			const approval = await sendChanglheToserver(computedChallenge, serverChallenge);

			if (!approval) {
				console.error(`Challenge verification failed for device: ${device.name}`);
				connectedDevice.cancelConnection();
				continue;
			}

			bleManager.cancelDeviceConnection(device.id);

			console.log(`Successfully connected and verified: ${device.name}`);
			connectedDevices.push(connectedDevice);
		} catch (error) {
			console.error(`Error during connection/verification for device: ${device.name}`, error);
			device.cancelConnection();
			continue;
		}

	}

	return connectedDevices;
}

