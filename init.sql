CREATE TABLE IF NOT EXISTS users (
    id INT AUTO_INCREMENT PRIMARY KEY,
    username VARCHAR(255) NOT NULL UNIQUE,
    password VARCHAR(255) NOT NULL
);

CREATE TABLE IF NOT EXISTS trips (
    id INT AUTO_INCREMENT PRIMARY KEY,
    user_id INT NOT NULL,
    start_location VARCHAR(255) DEFAULT NULL,
    end_location VARCHAR(255) DEFAULT NULL,
    price INT DEFAULT 0,
    trip_date DATETIME DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
);

CREATE TABLE IF NOT EXISTS trip_data (
    id INT AUTO_INCREMENT PRIMARY KEY,
    trip_id INT NOT NULL,
    position VARCHAR(255) NOT NULL,
    time_stamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (trip_id) REFERENCES trips(id) ON DELETE CASCADE
);

CREATE TABLE IF NOT EXISTS vehicles (
    id INT AUTO_INCREMENT PRIMARY KEY,
    vehicle_name VARCHAR(255),
    current_location VARCHAR(255),
    ammount_of_passengers INT DEFAULT 0,
    registered_at DATETIME DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE IF NOT EXISTS bluetooth_devices (
    id INT AUTO_INCREMENT PRIMARY KEY,
    vehicle_id INT NOT NULL,
    device_name VARCHAR(255),
    secret VARCHAR(255),
    FOREIGN KEY (vehicle_id) REFERENCES vehicles(id) ON DELETE CASCADE
);

INSERT INTO vehicles (vehicle_name, current_location, ammount_of_passengers) VALUES ('Metro', 'Sluseholmen St.', 0);
INSERT INTO bluetooth_devices (vehicle_id, device_name, secret) VALUES (1, 'ESP32-1111', 'very_SECRET_esp_1111');
INSERT INTO bluetooth_devices (vehicle_id, device_name, secret) VALUES (1, 'ESP32-2222', 'very_SECRET_esp_2222');






