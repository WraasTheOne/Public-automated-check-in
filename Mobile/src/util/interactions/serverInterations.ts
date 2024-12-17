import { useEffect, useState } from 'react';
import { BleManager, Device } from 'react-native-ble-plx';

import AsyncStorage from '@react-native-async-storage/async-storage';


//+----------------+--------------+------+-----+-------------------+-------------------+
//| Field          | Type         | Null | Key | Default           | Extra             |
//+----------------+--------------+------+-----+-------------------+-------------------+
//| id             | int          | NO   | PRI | NULL              | auto_increment    |
//| user_id        | int          | NO   | MUL | NULL              |                   |
//| start_location | varchar(255) | YES  |     | NULL              |                   |
//| end_location   | varchar(255) | YES  |     | NULL              |                   |
//| price          | int          | YES  |     | 0                 |                   |
//| trip_date      | datetime     | YES  |     | CURRENT_TIMESTAMP | DEFAULT_GENERATED |
//| finished_trip  | tinyint(1)   | YES  |     | 0                 |                   |
//+----------------+--------------+------+-----+-------------------+-------------------+


//export interface Journey {
//	start_location: string;
//	end_location: string;
//	price: number;
//	trip_date: Date;
//}
//mkae it as an interface

export interface Journey {
	start_location: string;
	end_location: string;
	price: number
	trip_date: string;
}


interface BleServerInteractions {
	GetChallengeFromServer: (espId: string) => Promise<string>;
	sendChanglheToserver: (hashedCahheld: string, challange: string) => Promise<boolean>;
	GetCheckInStatus: () => Promise<[boolean, number]>;
	GetJourneys: () => Promise<Journey[]>;
}

const GetChallengeFromServer = async (espId: string): Promise<string> => {
	//192.168.1.68:8050 /getChallenge/esp/:id
	//get only the last 4 digits of espId
	espId = espId.slice(-4);
	const token = await AsyncStorage.getItem('userToken');
	if (!token) {
		throw new Error('No token found');
	}
	const response = await fetch(`http://192.168.102.10:8050/getChallenge/esp/${espId}`, {
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
	const response = await fetch('http://192.168.102.10:8050/verifyChallenge', {
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

const GetCheckInStatus = async (): Promise<[boolean, number]> => {
	const token = await AsyncStorage.getItem('userToken');
	if (!token) {
		throw new Error('No token found');
	}
	const response = await fetch("http://192.168.102.10:8080/checkInStatus", {
		method: 'GET',
		headers: {
			'Content-Type': 'application/json',
			'Authorization': `${token}`
		},
	})
	const data = await response.json();
	if (response.status === 200) {

		return [data.checked_in, data.wallet];
	}

	return [false, 0];
};

const GetJourneys = async (): Promise<Journey[]> => {
	const token = await AsyncStorage.getItem('userToken');
	if (!token) {
		throw new Error('No token found');
	}
	const response = await fetch("http://192.168.102.10:8080/userjourneys", {
		method: 'GET',
		headers: {
			'Content-Type': 'application/json',
			'Authorization': `${token}`
		},
	})
	const data = await response.json();
	if (response.status === 200) {

		const journeys: Journey[] = data.journeys.map((journey: any) => {
			console.log(journey.price);
			return {
				start_location: journey.start_location,
				end_location: journey.end_location,
				price: journey.price,
				trip_date: new Date(journey.trip_date)
			}
		});

		return journeys
	}
	return [];

}


const server = (): BleServerInteractions => {
	return { GetChallengeFromServer, sendChanglheToserver, GetCheckInStatus, GetJourneys };
};

export default server;
