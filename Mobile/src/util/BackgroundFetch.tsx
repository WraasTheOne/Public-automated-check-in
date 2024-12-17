import * as BackgroundFetch from 'expo-background-fetch';
import * as TaskManager from 'expo-task-manager';
import serverinter from './interactions/serverInterations';


import AsyncStorage from '@react-native-async-storage/async-storage';


import { startBleScanAndConnect } from './BackgoudBle';



import * as Notifications from 'expo-notifications';
import * as Device from 'expo-device';

Notifications.setNotificationHandler({
	handleNotification: async () => ({
		shouldShowAlert: true,
		shouldPlaySound: true,
		shouldSetBadge: false,
	}),
});


const BACKGROUND_FETCH_TASK = 'BACKGROUND_FETCH_TASK';


const scheduleNotification = async (tite: string, body: string) => {
	await Notifications.scheduleNotificationAsync({
		content: {
			title: tite,
			body: body,
		},
		// Trigger notification after 10 seconds
		trigger: null,
	});
	console.log('Notification scheduled for 10 seconds from now.');
};


// Define the background fetch task
//
TaskManager.defineTask(BACKGROUND_FETCH_TASK, async () => {
	try {
		const { GetCheckInStatus, GetJourneys } = serverinter();
		console.log('Background fetch task executed');


		//const getFechedData = async () => {
		//	//getChallenge/esp/ESP32-1111
		//	const token = await AsyncStorage.getItem('userToken');
		//	const response = await fetch('http://192.168.1.68:8050/getChallenge/esp/ESP32-1111', {
		//		method: 'GET',
		//		headers: {
		//			'Content-Type': 'application/json',
		//			'Authorization': `${token}`
		//		},
		//	})
		//	const data = await response.json();
		//	if (response.status === 200) {
		//		return data.challenge;
		//	}
		//	throw new Error('Failed to get challenge', data);
		//}
		//for (let i = 0; i < 10; i++) {
		//	const data = await getFechedData();
		//	console.log(data);
		//}

		//if (Device.isDevice) {
		//	// Check for notification permissions
		//	let { status } = await Notifications.getPermissionsAsync();
		//	if (status !== 'granted') {
		//		const { status: newStatus } = await Notifications.requestPermissionsAsync();
		//		status = newStatus;
		//	}

		//	// If permissions are granted, schedule a local notification
		//	if (status === 'granted') {
		//		await Notifications.scheduleNotificationAsync({
		//			content: {
		//				title: 'Background Fetch Reminder',
		//				body: 'Remember: somring simring',
		//			},
		//			trigger: null,
		//		});
		//		console.log('Local notification scheduled from background task.');
		//
		const isRssied = await startBleScanAndConnect();
		if (isRssied) {
			const [status, wallet] = await GetCheckInStatus();
			if (status) {
				scheduleNotification('Checked In', 'You have been checked in!');
			} else {

			}

		}

		return BackgroundFetch.BackgroundFetchResult.NewData;
	} catch (error) {
		console.error('Error in background fetch task:', error);
		// Return failure
		return BackgroundFetch.BackgroundFetchResult.Failed;
	}
});

// Function to register the background fetch task
export const registerBackgroundFetchTask = async () => {
	try {
		const status = await BackgroundFetch.getStatusAsync();
		if (status === BackgroundFetch.BackgroundFetchStatus.Restricted) {
			console.warn('Background fetch is restricted');
			return;
		}

		const isRegistered = await TaskManager.isTaskRegisteredAsync(BACKGROUND_FETCH_TASK);
		if (!isRegistered) {
			await BackgroundFetch.registerTaskAsync(BACKGROUND_FETCH_TASK, {
				minimumInterval: 60, // 1 minute
				stopOnTerminate: false,
				startOnBoot: true,
			});
			console.log('Background fetch task registered');
		} else {
			console.log('Background fetch task already registered');
		}
	} catch (error) {
		console.error('Error registering background fetch task:', error);
	}
};
