import { useEffect } from "react";
import { useMemo, useState } from "react";
import {
	BleManager,
	Device,
} from "react-native-ble-plx";

import serverInterations from "./interactions/serverInterations";
import espInteractions from "./interactions/BleInterations";
import { connectToEspAndverify } from "./interactions/connectToEspAndverify";
import { getRssiFromDevises } from "./BleRssi";



import * as ExpoDevice from "expo-device";
interface BlueToothLowEnergy {
	startScan(): void;
	disconnectAllDevices(): void;
	alldevices: Device[];
	verifyiedList: Device[];
	isCheakdIn: boolean;
}

function useBle(): BlueToothLowEnergy {
	const bleManager = useMemo(() => new BleManager(), []);

	const [isCheakdIn, setIsCheakdIn] = useState(false);

	const [alldevices, setAllDevices] = useState<Device[]>([]);
	const [isScanning, setIsScanning] = useState(false);
	const [verifyiedList, setVerifiedList] = useState<Device[]>([]);

	//disconnect all divices
	const disconnectAllDevices = async () => {
		console.log("Disconnecting all devices...", verifyiedList.length);
		//map through the verified list and disconnect all the devices
		const disconnections = verifyiedList.map(async (device) => {
			try {
				console.log("Disconnecting device:", device.id);
				await device.cancelConnection();
			} catch (error) {
				console.error("Error disconnecting device:", device.id, error);
			}
		});

		await Promise.all(disconnections).then(() => {
			console.log("All devices disconnected");
		});

		setVerifiedList([]);
		setAllDevices([]);
		setIsCheakdIn(false);
	};

	const startScan = () => {
		setIsScanning(true);
		setAllDevices([]);
		bleManager.startDeviceScan(null, null, (error, device) => {
			if (error) {
				console.error("Error during scan:", error);
				return;
			}
			if (device?.name && device.name.startsWith("ESP32")) {
				setAllDevices((prevDevices) => {
					if (!prevDevices.some((d) => d.id === device.id)) {
						if (!verifyiedList.some((d) => d.id === device.id)) {
							return [...prevDevices, device];
						} else {
							console.log("ahhaha taht waht one of the devices");
						}

					}
					return prevDevices;
				});
			}
		});
		setTimeout(() => {
			console.log("Stopping scan...");
			bleManager.stopDeviceScan();
			setIsScanning(false);
		}, 3000);
	};

	useEffect(() => {
		if (!isScanning && alldevices.length > 0) {
			console.log("Connecting to devices and verifying...", verifyiedList);
			const connectAndVerify = async () => {
				const verifiedDevices = await connectToEspAndverify(bleManager, alldevices);
				if (verifiedDevices === null) {
					return;
				}
				console.log("Verified devices:", verifiedDevices.length);
				setVerifiedList(verifiedDevices);
			}
			connectAndVerify();
			setAllDevices([]); // Clear the device list after attempting connections
		}
	}, [alldevices, isScanning, bleManager]);

	useEffect(() => {

		if (verifyiedList.length === 0) {
			return;
		}

		const intervalId = setInterval(async () => {
			await getRssiFromDevises(verifyiedList);
			console.log("RSSIs:");
		}, 2000);

		return () => { console.log("Clearing RSSI monitoring interval..."); clearInterval(intervalId); };

	}, [verifyiedList]);

	return {
		startScan,
		alldevices,
		disconnectAllDevices,
		verifyiedList,
		isCheakdIn,
	};
}

export default useBle;
