import { PermissionsAndroid, Platform } from "react-native";
import * as ExpoDevice from "expo-device";

const getRequiredPermissions = async () => {
	const requestBleScanPermission = async () => {
		const result = await PermissionsAndroid.request(
			PermissionsAndroid.PERMISSIONS.BLUETOOTH_SCAN,
			{
				title: "Bluetooth Scan Permission",
				message:
					"This app needs access to Bluetooth scanning " +
					"so it can find BLE devices, even in the background.",
				buttonNeutral: "Ask Me Later",
				buttonNegative: "Cancel",
				buttonPositive: "OK",
			}
		);
		return result === PermissionsAndroid.RESULTS.GRANTED;
	};

	const requestBleConnectPermission = async () => {
		const result = await PermissionsAndroid.request(
			PermissionsAndroid.PERMISSIONS.BLUETOOTH_CONNECT,
			{
				title: "Bluetooth Connect Permission",
				message:
					"This app needs Bluetooth connect permission " +
					"to maintain connections with BLE devices.",
				buttonNeutral: "Ask Me Later",
				buttonNegative: "Cancel",
				buttonPositive: "OK",
			}
		);
		return result === PermissionsAndroid.RESULTS.GRANTED;
	};

	const requestFineLocationPermission = async () => {
		const result = await PermissionsAndroid.request(
			PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION,
			{
				title: "Fine Location Permission",
				message:
					"This app needs Fine Location permission " +
					"to detect BLE devices near you.",
				buttonNeutral: "Ask Me Later",
				buttonNegative: "Cancel",
				buttonPositive: "OK",
			}
		);
		return result === PermissionsAndroid.RESULTS.GRANTED;
	};

	const requestBackgroundLocationPermission = async () => {
		// Only required if API level >= 29 (Android 10+)
		if ((ExpoDevice.platformApiLevel ?? -1) < 29) {
			return true;
		}
		const result = await PermissionsAndroid.request(
			PermissionsAndroid.PERMISSIONS.ACCESS_BACKGROUND_LOCATION,
			{
				title: "Background Location Permission",
				message:
					"This app needs background location access " +
					"to scan for BLE devices even when the app isn't active.",
				buttonNeutral: "Ask Me Later",
				buttonNegative: "Cancel",
				buttonPositive: "OK",
			}
		);
		return result === PermissionsAndroid.RESULTS.GRANTED;
	};

	const requestPermissions = async () => {
		if (Platform.OS === "android") {
			const apiLevel = ExpoDevice.platformApiLevel ?? -1;
			if (apiLevel < 31) {
				// For Android < 31, we don't need BLE_SCAN and BLE_CONNECT permissions.
				// Just request Fine Location and then Background Location if needed.
				const fineGranted = await requestFineLocationPermission();
				if (!fineGranted) return false;

				const bgGranted = await requestBackgroundLocationPermission();
				return bgGranted;
			} else {
				// Android 31+ requires BLE_SCAN and BLE_CONNECT.
				const scanGranted = await requestBleScanPermission();
				if (!scanGranted) return false;

				const connectGranted = await requestBleConnectPermission();
				if (!connectGranted) return false;

				const fineGranted = await requestFineLocationPermission();
				if (!fineGranted) return false;

				const bgGranted = await requestBackgroundLocationPermission();
				return bgGranted;
			}
		} else {
			// iOS or other platforms
			return true;
		}
	};

	return requestPermissions();
};

export default getRequiredPermissions;
