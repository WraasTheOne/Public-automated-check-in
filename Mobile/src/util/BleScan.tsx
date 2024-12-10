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
		setIsCheakdIn(false);
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

	const disconnectDevice = async (device: Device) => {
		try {
			await device.cancelConnection();
			setVerifiedList((prevDevices) =>
				prevDevices.filter((d) => d.id !== device.id)
			);
		} catch (error) {
			console.error("Error disconnecting device:", device.id, error);
		}
	};

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
					console.error("Error reading RSSI from device:", verifiedEsp.id, error);
					disconnectDevice(verifiedEsp);

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
