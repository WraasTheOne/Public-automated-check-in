// context/AuthContext.tsx
import React, { createContext, useState, useEffect, ReactNode } from 'react';
import AsyncStorage from '@react-native-async-storage/async-storage';

type AuthContextType = {
	isLoggedIn: boolean;
	signIn: (token: string) => void;
	signOut: () => void;
};

export const AuthContext = createContext<AuthContextType>({
	isLoggedIn: false,
	signIn: () => { },
	signOut: () => { },
});

type Props = {
	children: ReactNode;
};

export const AuthProvider: React.FC<Props> = ({ children }) => {
	const [isLoggedIn, setIsLoggedIn] = useState<boolean>(false);
	const [isLoading, setIsLoading] = useState<boolean>(true);

	useEffect(() => {
		const checkLoginStatus = async () => {
			try {
				const token = await AsyncStorage.getItem('userToken');
				setIsLoggedIn(!!token);
			} catch (e) {
				console.error('Failed to load token');
			} finally {
				setIsLoading(false);
			}
		};

		checkLoginStatus();
	}, []);

	const signIn = async (token: string) => {
		try {
			await AsyncStorage.setItem('userToken', token);
			setIsLoggedIn(true);
		} catch (e) {
			console.error('Failed to save token');
		}
	};

	const signOut = async () => {
		try {
			await AsyncStorage.removeItem('userToken');
			setIsLoggedIn(false);
		} catch (e) {
			console.error('Failed to remove token');
		}
	};

	if (isLoading) {
		// You can return a splash screen or loader here
		return null;
	}

	return (
		<AuthContext.Provider value={{ isLoggedIn, signIn, signOut }}>
			{children}
		</AuthContext.Provider>
	);
};

