// screens/LoginScreen.tsx
import React, { useState, useContext } from 'react';
import {
	View,
	Text,
	TextInput,
	Button,
	StyleSheet,
	Alert
} from 'react-native';
import { AuthContext } from '../context/AuthContext';

const login = async (username: string, password: string) => {
	const response = await fetch('http://192.168.1.68:8080/login', {
		method: 'POST',
		headers: {
			'Content-Type': 'application/json'
		},
		body: JSON.stringify({
			username,
			password
		})
	});
	const data = await response.json();
	if (response.ok) {
		return data.token;
	}
	throw new Error(data.message);
};


const LoginScreen: React.FC = () => {
	const { signIn } = useContext(AuthContext);
	const [username, setUsername] = useState('');
	const [password, setPassword] = useState('');

	const handleLogin = async () => {  // Making handleLogin an async function
		try {
			login(username, password)
				.then((token) => {
					signIn(token);
				})
		} catch (e) {
			Alert.alert('Woring Please try again');
		}
	};

	return (
		<View style={styles.container}>
			<Text style={styles.title}>Login</Text>
			<TextInput
				placeholder="Username"
				style={styles.input}
				value={username}
				onChangeText={setUsername}
				autoCapitalize='none'
			/>
			<TextInput
				placeholder="Password"
				style={styles.input}
				value={password}
				onChangeText={setPassword}
				secureTextEntry
			/>
			<Button title="Login" onPress={handleLogin} />
		</View>
	);
};

export default LoginScreen;

const styles = StyleSheet.create({
	container: {
		flex: 1,
		justifyContent: 'center',
		padding: 20,
		backgroundColor: '#fff'
	},
	title: {
		fontSize: 32,
		marginBottom: 20,
		textAlign: 'center'
	},
	input: {
		height: 50,
		borderColor: '#ccc',
		borderWidth: 1,
		marginBottom: 15,
		paddingHorizontal: 10,
		borderRadius: 5
	},
});

