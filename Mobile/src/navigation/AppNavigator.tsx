// navigation/AppNavigator.tsx
import React, { useContext } from 'react';
import { NavigationContainer } from '@react-navigation/native';
import { createNativeStackNavigator } from '@react-navigation/native-stack';

import { AuthContext } from '../context/AuthContext';

// Import your screens
import LoginScreen from '../screens/LoginScreen';
import HomeScreen from '../screens/HomeScreen';

export type RootStackParamList = {
	Login: undefined;
	Home: undefined;
};

const Stack = createNativeStackNavigator<RootStackParamList>();

const AppNavigator = () => {
	const { isLoggedIn } = useContext(AuthContext);

	return (
		<NavigationContainer>
			<Stack.Navigator>
				{isLoggedIn ? (
					<Stack.Screen
						name="Home"
						component={HomeScreen}
						options={{ headerShown: false }}
					/>
				) : (
					<Stack.Screen
						name="Login"
						component={LoginScreen}
						options={{ headerShown: false }}
					/>
				)}
			</Stack.Navigator>
		</NavigationContainer>
	);
};

export default AppNavigator;

