import mysql.connector

class MySQLDatabase:
    def __init__(self, user, password, host, port, database):
        self.connection = mysql.connector.connect(
            user=user,
            password=password,
            host=host,
            port=port,
            database=database
        )
        self.cursor = self.connection.cursor(dictionary=True)

    def register_trip(self, user_id):
        query = "INSERT INTO trips (user_id) VALUES (%s)"
        try:
            self.cursor.execute(query, (user_id,))
            self.connection.commit()
            return self.cursor.lastrowid
        except mysql.connector.Error as e:
            raise Exception(f"Failed to register trip: {e}")

    def register_trip_data(self, trip_id, position):
        query = "INSERT INTO trip_data (trip_id, position) VALUES (%s, %s)"
        try:
            self.cursor.execute(query, (trip_id, position))
            self.connection.commit()
        except mysql.connector.Error as e:
            raise Exception(f"Failed to register trip data: {e}")

    def get_current_location(self, bluetooth_device_id):
        query = """
            SELECT v.current_location, v.ammount_of_passengers
            FROM vehicles v
            INNER JOIN bluetooth_devices bd ON v.id = bd.vehicle_id
            WHERE bd.id = %s
        """
        try:
            self.cursor.execute(query, (bluetooth_device_id,))
            result = self.cursor.fetchone()
            if not result:
                raise Exception(f"No vehicle found for Bluetooth device ID {bluetooth_device_id}")
            return result['current_location'], result['ammount_of_passengers']
        except mysql.connector.Error as e:
            raise Exception(f"Failed to get current location: {e}")

    def add_passenger_to_vehicle(self, bluetooth_device_id):
        query = """
            UPDATE vehicles
            SET ammount_of_passengers = ammount_of_passengers + 1
            WHERE id = (
                SELECT vehicle_id
                FROM bluetooth_devices
                WHERE id = %s
            )
        """
        try:
            self.cursor.execute(query, (bluetooth_device_id,))
            self.connection.commit()
        except mysql.connector.Error as e:
            raise Exception(f"Failed to add passenger to vehicle: {e}")

    def close(self):
        if self.cursor:
            self.cursor.close()
        if self.connection:
            self.connection.close()