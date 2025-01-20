// screens/HomeScreen.tsx
import React, { useState, useContext } from 'react';
import { useEffect } from 'react';
import { View, Text, Animated, Button, StyleSheet, Modal, TouchableOpacity, ScrollView } from 'react-native';
import { AuthContext } from '../context/AuthContext';
import AsyncStorage from '@react-native-async-storage/async-storage';
import useBle from '../util/BleScan';
import * as Notifications from 'expo-notifications';
import serverinter from "../util/interactions/serverInterations";

import JourneyPath from '../util/path/JourneyPath';

export interface Journey {
	start_location: string;
	end_location: string;
	price: number;
	trip_date: string;
}

Notifications.setNotificationHandler({
	handleNotification: async () => ({
		shouldShowAlert: true,
		shouldPlaySound: true,
		shouldSetBadge: false,
	}),
});

const HomeScreen: React.FC = () => {
	const { startScan } = useBle();


	const [scrollY] = useState(new Animated.Value(0));

	const { isCheckedIn, setIsCheckedIn } = useContext(AuthContext);
	const { signOut } = useContext(AuthContext);

	const { GetCheckInStatus, GetJourneys } = serverinter();
	const [journStatus, setJournStatus] = useState(false);
	const [journeys, setJourneys] = useState<Journey[]>([]);

	const [wallet, setWallet] = useState(0);
	const [colorChange, setColorChange] = useState("lightblue");

	useEffect(() => {
		//startScan();
		const getJourneys = async () => {
			const journeys = await GetJourneys();
			setJourneys(journeys);
			journeys.forEach((journey) => {
				console.log(journey);
			});

		}
		getJourneys();

	}, [isCheckedIn]);

	useEffect(() => {
		const checkInStatus = async () => {
			const [status, wallet] = await GetCheckInStatus();

			// Track status changes
			if (status !== isCheckedIn) {
				if (status) {
					scheduleNotification("Checked In", "You have been checked in!");
				} else {
					scheduleNotification("Checked Out", "You have been checked out!");
				}
				setIsCheckedIn(status); // Update state only when status changes
			}

			setWallet(wallet);

			if (status) {
				setJournStatus(true);
				setColorChange("lightgreen");
			} else {
				setJournStatus(false);
				setColorChange("lightblue");
			}
		};

		// Set up interval
		const interval = setInterval(() => {
			checkInStatus();
		}, 3000);

		return () => clearInterval(interval);
	}, [isCheckedIn]); // Depend on `isCheckedIn` to track changes

	const scheduleNotification = async (tite: string, body: string) => {
		await Notifications.scheduleNotificationAsync({
			content: {
				title: tite,
				body: body,
			},
			// Trigger notification after 10 seconds
			trigger: null,
		});
		console.log('Notification scheduled for 10 seconds from now.');
	};

	useEffect(() => {
		//startScan();
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

	}, []);  // The effect runs when `isCheckedIn` changes

	const handelJourney = () => {
		//flip the value of journStatus
		setJournStatus(!journStatus);
		if (journStatus) {
			setColorChange("lightblue");
		} else {
			setColorChange("lightgreen");
		}
	}

	return (
		<View style={styles.container}>
			<View style={styles.krText}>
				<Text style={{ fontWeight: 'bold' }}>Wallet: {wallet} KR</Text>
			</View>

			<TouchableOpacity
				style={{ position: 'absolute', right: 10, top: 10 }}
				onPress={signOut} >
				<Text> Logout </Text>
			</TouchableOpacity>
			{/* First Box */}
			<View style={styles.containerRow}>
				<Text style={styles.title}>Latest journeys:</Text>
                				<ScrollView>
					{journeys.map((journey, index) => (
						<View key={index} style={styles.itemContainer}>
							{/* Price on the left */}
							<Text style={styles.price}>Price: {journey.price} Kr</Text>

							{/* SVG Path */}
							<JourneyPath width={300} height={50} color="#48A2B7" />

							{/* Start and End Labels */}
							<View style={styles.labelContainer}>
								<View style={styles.label}>
									<Text style={styles.topLabel}>{journey.start_location}</Text>
									<Text style={styles.bottomLabel}>start</Text>
								</View>
								<View style={styles.label}>
									<Text style={styles.topLabel}>{journey.end_location}</Text>
									<Text style={styles.bottomLabel}>end</Text>
								</View>
							</View>
						</View>
					))}
				</ScrollView>

			</View>
			{/* Second Box */}
			<View style={[styles.constSrek, { backgroundColor: colorChange }]}>
				<Text style={styles.constSrekText}>
					{journStatus ? "Journey started" : "Journey not started"}
				</Text>
				{!journStatus ? (
					<TouchableOpacity
						style={[styles.greenButton, styles.button]}
						onPress={handelJourney}
					>
						<Text style={styles.buttonText}>Start Journey</Text>
					</TouchableOpacity>
				) : (
					<TouchableOpacity
						style={[styles.redButton, styles.button]}
						onPress={handelJourney}
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
	krText: {
		borderRadius: 10,
		backgroundColor: '#fff',
		shadowColor: '#000',
		shadowOffset: { width: 0, height: 2 },
		shadowOpacity: 0.2,
		shadowRadius: 4,
		elevation: 2,


		right: 100,
		marginBottom: 20,
		flexDirection: 'row',
		justifyContent: 'space-between',
		padding: 20,
	},
	itemContainer: {
		marginBottom: 20,
		alignItems: 'center',
		position: 'relative',
		backgroundColor: '#fff',
		borderRadius: 10,
		padding: 20,
		shadowColor: '#000',
		shadowOffset: { width: 0, height: 2 },
		shadowOpacity: 0.2,
		shadowRadius: 4,
		elevation: 2,
	},
	price: {
		position: 'absolute',
		top: '20%',
		fontSize: 14,
		fontWeight: 'bold',
		color: '#333',
	},
	labelContainer: {
		width: '100%',
		flexDirection: 'row',
		justifyContent: 'space-between',
		marginTop: -10,
	},
	label: {
		alignItems: 'center',
	},
	topLabel: {
		fontSize: 12,
		fontWeight: 'bold',
		marginBottom: 4,
		color: '#555',
	},
	bottomLabel: {
		fontSize: 12,
		color: '#777',
	},

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

