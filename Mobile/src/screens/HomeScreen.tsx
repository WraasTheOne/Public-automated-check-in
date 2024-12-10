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
import getRequiredPermissions from '../util/Promisitons';
import AsyncStorage from '@react-native-async-storage/async-storage';
import useBle from '../util/BleScan';

const HomeScreen: React.FC = () => {
	const {
		startScan,
		alldevices,
		disconnectAllDevices,
		verifyiedList,
		correntDevice,
		isCheakdIn

	} = useBle();

	const { signOut } = useContext(AuthContext);
	const [journStatus, setJournStatus] = useState(false);
	const [colorChange, setColorChange] = useState("lightblue");

	useEffect(() => {
		if (isCheakdIn) {
			setJournStatus(true);
			setColorChange("lightgreen");
		} else {
			setJournStatus(false);
			setColorChange("lightblue");
		}

	}, [isCheakdIn]);

	useEffect(() => {
		const starTtime = setTimeout(() => {
			if (!isCheakdIn) {
				startScan();
			}
		}, 10000);

		return () => { clearTimeout(starTtime); }
	}, [isCheakdIn]);


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

