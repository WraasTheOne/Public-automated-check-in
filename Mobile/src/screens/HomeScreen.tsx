// screens/HomeScreen.tsx
import React, { useState, useContext } from 'react';
import { useEffect } from 'react';
import {
	View,
	Text,
	Button,
	StyleSheet,
	Modal,
	TouchableOpacity
} from 'react-native';
import { AuthContext } from '../context/AuthContext';
import { Alert } from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';
import useBle from '../util/BleScan';
//add notification
import * as Notifications from 'expo-notifications';
import * as Device from 'expo-device';

Notifications.setNotificationHandler({
	handleNotification: async () => ({
		shouldShowAlert: true,
		shouldPlaySound: true,
		shouldSetBadge: false,
	}),
});

const HomeScreen: React.FC = () => {
	const {
		startScan,
		disconnectAllDevices,
		isCheakdIn

	} = useBle();

	const { signOut } = useContext(AuthContext);
	const [journStatus, setJournStatus] = useState(false);
	const [colorChange, setColorChange] = useState("lightblue");
	const [permissionStatus, setPermissionStatus] = useState('');

	useEffect(() => {

		const checkIfCheakdIn = async () => {
			const value = await AsyncStorage.getItem("BackIsCheakdIn");
			if (value) {
				setJournStatus(true);
				setColorChange("lightgreen");
				await AsyncStorage.removeItem("BackIsCheakdIn");
			}
		}
		checkIfCheakdIn();

		if (isCheakdIn) {
			setJournStatus(true);
			setColorChange("lightgreen");
		} else {
			setJournStatus(false);
			setColorChange("lightblue");
		}

	}, [isCheakdIn]);

	const scheduleNotification = async () => {
		await Notifications.scheduleNotificationAsync({
			content: {
				title: 'Reminder',
				body: 'Remember: somring simring',
			},
			// Trigger notification after 10 seconds
			trigger: null,
		});
		console.log('Notification scheduled for 10 seconds from now.');
	};


	useEffect(() => {
		startScan();
		//(async () => {
		//	// Only request permissions on a physical device
		//	if (Device.isDevice) {
		//		const { status } = await Notifications.getPermissionsAsync();
		//		if (status !== 'granted') {
		//			const { status: newStatus } = await Notifications.requestPermissionsAsync();
		//			setPermissionStatus(newStatus);
		//		} else {
		//			setPermissionStatus(status);
		//		}
		//	} else {
		//		console.log('Must use physical device for Push Notifications');
		//	}
		//})();

	}, []);  // The effect runs when `isCheakdIn` changes

	const handelStartJourney = () => {
		startScan();
		setJournStatus(true);
	}

	const handelEndJourney = () => {
		disconnectAllDevices();
		setJournStatus(false);
	}


	return (
		<View style={styles.container}>
			<TouchableOpacity
				style={{ position: 'absolute', right: 10, top: 10 }}
				onPress={signOut} >
				<Text> Logout </Text>
			</TouchableOpacity>
			{/* First Box */}
			<View style={styles.containerRow}>
				<Text style={styles.title}>Latest journeys:</Text>

				<TouchableOpacity
					style={[styles.greenButton, styles.button]}
					onPress={() => scheduleNotification()}
				>
					<Text style={styles.buttonText}>press this</Text>
				</TouchableOpacity>
			</View>
			{/* Second Box */}
			<View style={[styles.constSrek, { backgroundColor: colorChange }]}>
				<Text style={styles.constSrekText}>
					{!journStatus
						? 'Ready to start your journey?'
						: isCheakdIn
							? 'You are checked in'
							: 'You are checked out'}
				</Text>
				{!journStatus ? (
					<TouchableOpacity
						style={[styles.greenButton, styles.button]}
						onPress={handelStartJourney}
					>
						<Text style={styles.buttonText}>Start Journey</Text>
					</TouchableOpacity>
				) : (
					<TouchableOpacity
						style={[styles.redButton, styles.button]}
						onPress={handelEndJourney}
					>
						<Text style={styles.buttonText}>End Journey</Text>
					</TouchableOpacity>
				)}
			</View>
		</View >
	);
};

export default HomeScreen;

const styles = StyleSheet.create({
	container: {
		flex: 1,
		flexDirection: 'column', // Default is column, so this line is optional
		justifyContent: 'center', // Centers vertically
		alignItems: 'center', // Centers horizontally
	},

	containerRow: {
		alignItems: 'center', // Center content horizontally
		width: '80%',
		height: '65%',
		borderRadius: 10,
		marginBottom: 20, // Space between boxes
	},
	// Second box styles
	constSrek: {
		justifyContent: 'center', // Center content vertically
		alignItems: 'center', // Center content horizontally
		backgroundColor: 'lightblue',
		padding: 20,
		width: '80%',
		height: '20%',
		borderRadius: 10,
	},
	button: {
		width: '80%',
		paddingVertical: 15,
		borderRadius: 25,
		alignItems: 'center',
		justifyContent: 'center',
		shadowColor: '#000',
		shadowOffset: { width: 0, height: 3 },
		shadowOpacity: 0.3,
		shadowRadius: 5,
		elevation: 5,
	},
	greenButton: {
		backgroundColor: '#28a745', // Cool green
	},
	constSrekText: {
		fontSize: 20,
		fontWeight: 'bold',
		color: '#333',
		marginBottom: 15,
	},
	redButton: {
		backgroundColor: '#dc3545', // Cool red
	},
	title: {
		fontSize: 24,
		fontWeight: 'bold',
		marginBottom: 20,
	},
	buttonText: {
		color: '#fff',
		fontSize: 19,
		fontWeight: 'bold',
	},
});

