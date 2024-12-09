import { useEffect } from "react";
import { useMemo, useState } from "react";
import { PermissionsAndroid, Platform } from "react-native";
import {
	BleError,
	BleManager,
	Characteristic,
	Device,
} from "react-native-ble-plx";

import serverInterations from "./interactions/serverInterations";
import espInteractions from "./interactions/BleInterations";

import * as ExpoDevice from "expo-device";
interface BlueToothLowEnergy {
	startScan(): void;
	disconnectAllDevices(): void;
	alldevices: Device[];
	verifyiedList: Device[];
	correntDevice: Device | null;
	isCheakdIn: boolean;
}

function useBle(): BlueToothLowEnergy {
	const bleManager = useMemo(() => new BleManager(), []);
	const [correntDevice, setCorrentDevice] = useState<Device | null>(null);

	const [isCheakdIn, setIsCheakdIn] = useState(false);

	const [alldevices, setAllDevices] = useState<Device[]>([]);
	const [isScanning, setIsScanning] = useState(false);
	const [verifyiedList, setVerifiedList] = useState<Device[]>([]);
	const [isVerified, setIsVerified] = useState(false);

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
	};

	const startScan = () => {
		setAllDevices([]);
		setIsScanning(true);

		bleManager.startDeviceScan(null, null, (error, device) => {
			if (error) {
				console.error("Error during scan:", error);
				return;
			}

			console.log("Starting BLE scan...");
			if (device?.name && device.name.startsWith("ESP32")) {
				setAllDevices((prevDevices) => {
					if (!prevDevices.some((d) => d.id === device.id)) {
						return [...prevDevices, device];
					}
					return prevDevices;
				});
			}
		});
		// Stop scanning after 10 seconds
		setTimeout(() => {
			console.log("Stopping scan...");
			bleManager.stopDeviceScan();
			setIsScanning(false);
		}, 2000);
	};

	const [count, setCount] = useState(0);

	//	useEffect(() => {
	//		const checkTheDevices = async () => {
	//
	//			//craete a list of rssi taht will be sent to the server
	//			const rssiList: number[] = [];
	//			verifyiedList.forEach(async (verifiedEsp) => {
	//				try {
	//					console.log("Reading RSSI from device:", verifiedEsp);
	//					const dviceRssi = await verifiedEsp.readRSSI();
	//					if (dviceRssi.rssi) {
	//						rssiList.push(dviceRssi.rssi);
	//					}
	//				} catch (error) {
	//					console.error("Error reading RSSI:", error);
	//				}
	//			}
	//			);
	//			//TODO: send the rssi list to the server
	//			console.log("list is sending to the server:", rssiList);
	//		}
	//
	//		if (isVerified) {
	//
	//			const id = setInterval(() => {
	//				console.log("Interval, count:", count);
	//				checkTheDevices();
	//				setCount((prev) => prev + 1);
	//			}, 1000);
	//
	//			// Clean up function to clear the interval when the component unmounts.
	//			return () => clearInterval(id);
	//		}
	//
	//	}, [count, isVerified]);
	//

	useEffect(() => {
		if (!isScanning && alldevices.length > 0) {
			const connectDevices = async () => {
				const connections = alldevices.map(async (device) => {
					try {
						console.log("Connecting to device:", device.id);
						const espInteract = espInteractions(bleManager, device);
						const getToken = await serverInterations().getToken();
						const connectedDevice = await espInteract.sendToken(getToken);
						const token = await espInteract.receiveToken();
						// TODO:			if the token
						//					is not that  the correct token skip the device
						console.log("Token received:", token);
						return connectedDevice;
					} catch (error) {
						console.error("Error connecting to device:", error);
						return null;
					}
				});

				const connectedDevices = await Promise.all(connections);
				setVerifiedList((prevDevices) => [
					...prevDevices,
					...connectedDevices.filter((dev) => dev !== null),
				]);
			};

			connectDevices();
			setAllDevices([]); // Clear the device list after attempting connections
		}
	}, [alldevices, isScanning, bleManager]);

	useEffect(() => {
		if (verifyiedList.length === 0) {
			return;
		}
		console.log("Starting RSSI monitoring interval...");

		const intervalId = setInterval(async () => {
			console.log("Checking RSSI for connected devices...");
			let highestRssi: number = -100;
			for (const verifiedEsp of verifyiedList) {
				try {
					const updatedDevice = await verifiedEsp.readRSSI();
					//reaber rssi can be null..
					if (updatedDevice.rssi === null) {
						console.error("RSSI is null for device:", verifiedEsp.name);
						continue;
					}
					if (updatedDevice.rssi > highestRssi) {
						highestRssi = updatedDevice.rssi;
						console.log("Highest RSSI:", highestRssi);
					}
					console.log("cheack in status:", isCheakdIn);
					console.log(
						"RSSI:",
						updatedDevice.rssi
					);
				} catch (error) {
					console.error("Error reading RSSI from device:", verifiedEsp.name, error);
				}
			}
			if (highestRssi > -50) {
				setIsCheakdIn(true);
			} else {
				setIsCheakdIn(false);
			}
		}, 2000);

		return () => {
			console.log("Clearing RSSI monitoring interval...");
			clearInterval(intervalId);
		}
	}, [verifyiedList]);

	return {
		startScan,
		alldevices,
		disconnectAllDevices,
		verifyiedList,
		correntDevice,
		isCheakdIn,
	};
}


export default useBle;
