import { BleManager, Device, Characteristic } from 'react-native-ble-plx';
import base64 from "react-native-base64";


interface BleInteractions {
	ConnetAndSendChallange: (challange: string, bleManager: BleManager, device: Device) => Promise<Device | null>;
	readChallange: (connectedDevice: Device) => Promise<string>;
}


function espInteractions(): BleInteractions {
	const SERVICE_UUID = '4fafc201-1fb5-459e-8fcc-c5c9c331914b';
	const CHARACTERISTIC_UUID = 'beb5483e-36e1-4688-b7f5-ea07361b26a8';

	const ConnetAndSendChallange = async (challange: string, bleManager: BleManager, device: Device): Promise<Device | null> => {
		try {
			// Connect if needed and discover all services and characteristics
			const connectedDevice = await bleManager.connectToDevice(device.id);
			await connectedDevice.discoverAllServicesAndCharacteristics();

			// Write the token to the characteristic (must be base64 encoded)
			await connectedDevice.writeCharacteristicWithResponseForService(
				SERVICE_UUID,
				CHARACTERISTIC_UUID,
				base64.encode(challange)
			);

			return connectedDevice
		} catch (error) {
			console.error('failed to send Challange:', error);
			return null;
		}
	};

	const readChallange = async (connectedDevice: Device): Promise<string> => {
		try {

			await connectedDevice.discoverAllServicesAndCharacteristics();
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

	return {
		ConnetAndSendChallange,
		readChallange
	};
}

export default espInteractions;
