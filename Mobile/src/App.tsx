// App.tsx
import React from 'react';
import { useContext } from 'react';
//useEffect is a hook that allows you to perform side effects in function components
import { useEffect } from 'react';
import { AuthProvider, AuthContext } from './context/AuthContext';
import { registerBackgroundFetchTask } from './util/BackgroundFetch';
import AppNavigator from './navigation/AppNavigator';
import getRequiredPermissions from './util/Promisitons';
import { Alert } from 'react-native';
import * as Notifications from 'expo-notifications';


export default function App() {
	useEffect(() => {


		const requestPermissions = async () => {
			//requesting permissions
			//
			const { status } = await Notifications.requestPermissionsAsync();
			if (status !== 'granted') {
				alert('No notification permissions!');
			}

			const granted = await getRequiredPermissions();

			if (granted) {
				registerBackgroundFetchTask();
			}

			else {
				Alert.alert('Permission Denied', 'Please allow the required permissions to proceed do it in settings');
			}

		};

		requestPermissions();

	}, []);




	return (
		<AuthProvider>
			<AppNavigator />
		</AuthProvider>
	);
}

