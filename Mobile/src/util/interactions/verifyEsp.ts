import { Device } from "react-native-ble-plx";

class VerifiedEsp {
	rssi: number;
	userId: string;
	device: Device

	// Constructor for initializing the properties
	constructor(rssi: number, userId: string, device: Device) {
		this.rssi = rssi;
		this.userId = userId;
		this.device = device;
	}

}

export default VerifiedEsp;
