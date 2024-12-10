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


const LoginScreen: React.FC = () => {
	const { signIn } = useContext(AuthContext);
	const [username, setUsername] = useState('');
	const [password, setPassword] = useState('');

	const handleLogin = () => {
		if (username === 'user' && password === 'Pass') {
			signIn('dommy-token');
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

