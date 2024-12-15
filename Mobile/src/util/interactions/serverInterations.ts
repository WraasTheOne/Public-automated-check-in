import { useEffect, useState } from 'react';
import { BleManager, Device } from 'react-native-ble-plx';

import AsyncStorage from '@react-native-async-storage/async-storage';

interface BleServerInteractions {
	GetChallengeFromServer: (espId: string) => Promise<string>;
	sendChanglheToserver: (hashedCahheld: string, challange: string) => Promise<boolean>;
}

const GetChallengeFromServer = async (espId: string): Promise<string> => {
	//192.168.1.68:8050 /getChallenge/esp/:id
	//get only the last 4 digits of espId
	espId = espId.slice(-4);
	const token = await AsyncStorage.getItem('userToken');
	if (!token) {
		throw new Error('No token found');
	}
	const response = await fetch(`http://192.168.1.68:8050/getChallenge/esp/${espId}`, {
		method: 'GET',
		headers: {
			'Content-Type': 'application/json',
			'Authorization': `${token}`
		},
	})
	const data = await response.json();
	if (response.status === 200) {
		return data.challenge;
	}
	throw new Error('Failed to get challenge', data);


};

const sendChanglheToserver = async (computedChallange: string, serverChallange: string): Promise<boolean> => {
	const token = await AsyncStorage.getItem('userToken');
	const response = await fetch('http://192.168.1.68:8050/verifyChallenge', {
		method: 'POST',
		headers: {
			'Content-Type': 'application/json',
			'Authorization': `${token}`
		},
		body: JSON.stringify({
			challenge: serverChallange,
			computed_hmac: computedChallange
		})
	});
	const data = await response.json();
	console.log(data);
	if (response.status === 200) {
		return true;
	}
	return false;
};


const server = (): BleServerInteractions => {
	return { GetChallengeFromServer, sendChanglheToserver };
};

export default server;
