import * as BackgroundFetch from 'expo-background-fetch';
import * as TaskManager from 'expo-task-manager';


import AsyncStorage from '@react-native-async-storage/async-storage';

import { startBleScanAndConnect } from './BackgoudBle';

const BACKGROUND_FETCH_TASK = 'BACKGROUND_FETCH_TASK';

// Define the background fetch task
//
TaskManager.defineTask(BACKGROUND_FETCH_TASK, async () => {
	try {
		console.log('Background fetch task executed');

		const userToken = await AsyncStorage.getItem('userToken');
		console.log('User token:', userToken);

		const devices = await startBleScanAndConnect();

		console.log('Connected devices:', devices);

		// Return success
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
