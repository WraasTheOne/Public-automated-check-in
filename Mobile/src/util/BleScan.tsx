import { useMemo, useState, useEffect, useContext } from "react";
import AsyncStorage from "@react-native-async-storage/async-storage";
import {
	BleManager,
	Device,
} from "react-native-ble-plx";

import { connectToEspAndverify } from "./interactions/connectToEspAndverify";
import serverinter from "./interactions/serverInterations";
import getRssiFromDetruments from "./BleRssi";
import { AuthContext } from '../context/AuthContext';
import * as ExpoDevice from "expo-device";

interface BlueToothLowEnergy {
	startScan: () => void;
}

function useBle(): BlueToothLowEnergy {
	const { connectForSeming, streeamRssiToServer } = getRssiFromDetruments();
	const { GetCheckInStatus } = serverinter();

	const bleManager = useMemo(() => new BleManager(), []);
	//get teh check in status
	const { setIsCheckedIn } = useContext(AuthContext);

	const [isScanning, setIsScanning] = useState(false);
	const [alldevices, setAllDevices] = useState<Device[]>([]);
	const [verifyiedList, setVerifiedList] = useState<Device[]>([]);
	const [redyToScan, setRedyToScan] = useState(false);

	useEffect(() => {

		startScan();

	}, []);



	const startScan = () => {
		console.log("We are scanning for devices...");
		setIsScanning(true);
		setAllDevices([]);
		bleManager.startDeviceScan(null, null, (error, device) => {


			if (error) {
				console.error("Error during scan:", error);
				return;
			}

			if (device?.name && device.name.startsWith("ESP32")) {
                console.log("Device found", device.name);
				setAllDevices((prevDevices) => {
					if (!prevDevices.some((d) => d.id === device.id)) {
						if (!verifyiedList.some((d) => d.id === device.id)) {
							return [...prevDevices, device];
						}
					}
					return prevDevices;
				});
			}
		});
		setTimeout(() => {
			console.log("Stopping scan...");

			if (alldevices.length === 0) {
				console.log("No devices found");
				setIsScanning(true);
			}
			bleManager.stopDeviceScan();
			setIsScanning(false);
		}, 3000)
	};

	useEffect(() => {

		if (redyToScan) {
			setRedyToScan(false);
			setTimeout(() => { startScan(); }, 5000);
		}

	}, [redyToScan]);


	const disconnectAllDevices = async (devices: Device[]) => {
		for (const device of devices) {
			try {
				console.log(`Disconnecting from device: ${device.name}`);
				await bleManager.cancelDeviceConnection(device.id);
			} catch (error) {
				console.log("Failed to disconnect from device:", error);
			}
		}
	}

	useEffect(() => {
		if (!isScanning && alldevices.length > 0) {
			console.log("Connecting to devices and verifying...", verifyiedList);
			const connectAndVerify = async () => {
				if (alldevices.length === 0) {
					return;
				}
				const verifiedDevices = await connectToEspAndverify(bleManager, alldevices);

				if (verifiedDevices === null) {
					return;
				}
				console.log("Verified devices:", verifiedDevices.length);
				setVerifiedList((prevDevices) => {
					return [...prevDevices, ...verifiedDevices];
				});

			}
			connectAndVerify();
			setAllDevices([]);
		}

	}, [alldevices, isScanning]);

	useEffect(() => {

		if (verifyiedList.length === 0) {
			return;
		}

		const getRssi = async () => {
			const connectedDevices = await connectForSeming(verifyiedList, bleManager);
			if (connectedDevices.length < 1) {
				return;
			}
			await streeamRssiToServer(connectedDevices)
				.finally(() => {
					disconnectAllDevices(connectedDevices);
					setVerifiedList([]);
					setRedyToScan(true);
					console.log("RSSI streaming complete.");
				});
		};

		getRssi();
	}, [verifyiedList]);

	return {
		startScan,
	};
}

export default useBle;
