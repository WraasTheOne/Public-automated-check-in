// context/AuthContext.tsx
import React, { createContext, useState, useEffect, ReactNode } from 'react';
import AsyncStorage from '@react-native-async-storage/async-storage';

type AuthContextType = {
	isCheckedIn: boolean;
	setIsCheckedIn: (value: boolean) => void;
	isLoggedIn: boolean;
	signIn: (token: string) => void;
	signOut: () => void;
};

//        NOTE:     thies vlaues are esentially 
export const AuthContext = createContext<AuthContextType>({
	isCheckedIn: false,
	isLoggedIn: false,
	setIsCheckedIn: () => { },
	signIn: () => { },
	signOut: () => { },
});

type Props = {
	children: ReactNode;
};

export const AuthProvider: React.FC<Props> = ({ children }) => {
	const [isCheckedIn, setIsCheckedIn] = useState<boolean>(false);
	const [isLoggedIn, setIsLoggedIn] = useState<boolean>(false);
	useEffect(() => {
		const checkLoginStatus = async () => {
			try {
				const token = await AsyncStorage.getItem('userToken');
				setIsLoggedIn(!!token);
			} catch (e) {
				console.error('Failed to load token');
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

	return (
		<AuthContext.Provider value={{ isLoggedIn, signIn, signOut, isCheckedIn, setIsCheckedIn }}>
			{children}
		</AuthContext.Provider>
	);
};

