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

			const serverChallenge = await GetChallengeFromServer(device.name);
			if (!serverChallenge) {
				continue;
			}
			const connectedDevice = await ConnetAndSendChallange(serverChallenge, bleManager, device);
			if (!connectedDevice) {
				console.log("Failed to connect to device ", device.name);
				continue;
			}
			const computedChallenge = await readChallange(connectedDevice);
			const approval = await sendChanglheToserver(computedChallenge, serverChallenge);

			if (!approval) {
				console.log("Server did not approved ", device.name);
				connectedDevice.cancelConnection();
				continue;
			}

			bleManager.cancelDeviceConnection(device.id);

			connectedDevices.push(connectedDevice);


		} catch (error) {
			console.error("Failed to connect to device: ", device.name, error);
			device.cancelConnection();
			continue;
		}

	}

	return connectedDevices;
}

