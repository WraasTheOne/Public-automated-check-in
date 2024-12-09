// App.tsx
import React from 'react';
//useEffect is a hook that allows you to perform side effects in function components
import { useEffect } from 'react';
import { AuthProvider } from './context/AuthContext';
import { registerBackgroundFetchTask } from './util/BackgroundFetch';
import AppNavigator from './navigation/AppNavigator';


export default function App() {

	useEffect(() => {
		registerBackgroundFetchTask();

	}, []);

	return (
		<AuthProvider>
			<AppNavigator />
		</AuthProvider>
	);
}

