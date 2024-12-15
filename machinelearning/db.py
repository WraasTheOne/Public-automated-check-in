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

    def get_checkin_status(self, user_id):
        query = "SELECT is_checked_in FROM users WHERE id = %s"
        try:
            self.cursor.execute(query, (user_id,))
            result = self.cursor.fetchone()
            if not result:
                raise Exception(f"No user found for user ID {user_id}")
            return result['is_checked_in']
        except mysql.connector.Error as e:
            raise Exception(f"Failed to get check-in status: {e}")

    def update_checkin_status(self, user_id, status):
        ##i use true and false instead of 1 and 0
        query = "UPDATE users SET is_checked_in = %s WHERE id = %s"
        try:
            self.cursor.execute(query, (status, user_id))
            self.connection.commit()
        except mysql.connector.Error as e:
            raise Exception(f"Failed to update check-in status: {e}")

    def register_trip(self, user_id, position):
        query = "INSERT INTO trips (user_id, start_location) VALUES (%s, %s)"
        try:
            self.cursor.execute(query, (user_id, position))
            self.connection.commit()
            return self.cursor.lastrowid
        except mysql.connector.Error as e:
            raise Exception(f"Failed to register trip: {e}")


    def register_trip_data(self,  position):
        query = """
            INSERT INTO trip_data (trip_id, position)
            VALUES (
                (SELECT id FROM trips ORDER BY trip_date DESC LIMIT 1),
                %s 
            )
        """
        try:
            self.cursor.execute(query, (position,))
            self.connection.commit()
        except mysql.connector.Error as e:
            raise Exception(f"Failed to register trip data: {e}")


    def get_current_location(self, bluetooth_device_name_id):
        query = """
            SELECT v.current_location, v.ammount_of_passengers
            FROM vehicles v
            INNER JOIN bluetooth_devices bd ON v.id = bd.vehicle_id
            WHERE bd.device_id_name = %s
        """
        try:
            self.cursor.execute(query, (bluetooth_device_name_id,))
            result = self.cursor.fetchone()
            if not result:
                raise Exception(f"No vehicle found for Bluetooth device name ID {bluetooth_device_name_id}")
            return result['current_location'], result['ammount_of_passengers']
        except mysql.connector.Error as e:
            raise Exception(f"Failed to get current location: {e}")

    def set_transport_status(self, user_id, status):
        query = "UPDATE users SET in_transport = %s WHERE id = %s"
        try:
            self.cursor.execute(query, (status, user_id))
            self.connection.commit()
        except mysql.connector.Error as e:
            raise Exception(f"Failed to set transport status: {e}")

    def get_transport_status(self, user_id):
        query = "SELECT in_transport FROM users WHERE id = %s"
        try:
            self.cursor.execute(query, (user_id,))
            result = self.cursor.fetchone()
            if not result:
                raise Exception(f"No user found for user ID {user_id}")
            return result['is_in_transport']
        except mysql.connector.Error as e:
            raise Exception(f"Failed to get transport status: {e}")

    def add_passenger_to_vehicle(self, bluetooth_device_name_id):
        query = """
            UPDATE vehicles
            SET ammount_of_passengers = ammount_of_passengers + 1
            WHERE id = (
                SELECT vehicle_id
                FROM bluetooth_devices
                WHERE  device_id_name = %s
            )
        """
        try:
            self.cursor.execute(query, (bluetooth_device_name_id,))
            self.connection.commit()
        except mysql.connector.Error as e:
            raise Exception(f"Failed to add passenger to vehicle: {e}")

    def get_user_id(self, user_id):
        query = "SELECT user from users WHERE id = %s"
        try:
            self.cursor.execute(query, (user_id,))
            result = self.cursor.fetchone()
            print(result)
            if not result:
                raise Exception(f"No user found for user ID {user_id}")
            return result['id']
        except mysql.connector.Error as e:
            raise Exception(f"Failed to get user ID: {e}")

    def close(self):
        if self.cursor:
            self.cursor.close()
        if self.connection:
            self.connection.close()
