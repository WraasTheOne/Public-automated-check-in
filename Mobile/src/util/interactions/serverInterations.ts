import { useEffect, useState } from 'react';
import { BleManager, Device } from 'react-native-ble-plx';

interface bleServerInteractions {
	getToken: () => Promise<string>;
	sendResponeToken: (token: string) => Promise<boolean>;
	sendRssis: (rssis: number[], userId: string) => Promise<boolean>;
}

function server(): bleServerInteractions {

	const getToken = async () => {
		return "token";
	}

	const sendResponeToken = async (token: string) => {
		if (!!token) {
			return true;
		}
		return false;
	}

	const sendRssis = async (rssis: number[], userId: string) => {

		if (rssis.length > 0) {
			console.log("USERID", userId);
			console.log("RSSIS", rssis);
			return false;
		}
		return true;
	}
	return { getToken, sendResponeToken, sendRssis };
}

export default server;
