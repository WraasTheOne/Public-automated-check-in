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

export default function App() {
	const { setIsReadyForBleScan } = useContext(AuthContext);

	useEffect(() => {
		const requestPermissions = async () => {
			const granted = await getRequiredPermissions();

			if (granted) {
				registerBackgroundFetchTask();
				setIsReadyForBleScan(true);
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

