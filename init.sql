
-- MySQL dump 10.13  Distrib 8.0.40, for Linux (x86_64)
--
-- Host: localhost    Database: userdb
-- ------------------------------------------------------
-- Server version       8.0.40

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!50503 SET NAMES utf8mb4 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `bluetooth_devices`
--

DROP TABLE IF EXISTS `bluetooth_devices`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `bluetooth_devices` (
  `id` int NOT NULL AUTO_INCREMENT,
  `device_id_name` int DEFAULT NULL,
  `registered_at` datetime DEFAULT CURRENT_TIMESTAMP,
  `vehicle_id` int NOT NULL,
  `secret` varchar(255) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `fk_vehicle_id` (`vehicle_id`),
  CONSTRAINT `fk_vehicle_id` FOREIGN KEY (`vehicle_id`) REFERENCES `vehicles` (`id`) ON DELETE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=3 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `bluetooth_devices`
--

LOCK TABLES `bluetooth_devices` WRITE;
/*!40000 ALTER TABLE `bluetooth_devices` DISABLE KEYS */;
INSERT INTO `bluetooth_devices` VALUES (1,2222,'2024-12-12 20:21:58',1,'very_SECRET_esp_2222'),(2,1111,'2024-12-12 20:22:24',1,'very_SECRET_esp_1111');
/*!40000 ALTER TABLE `bluetooth_devices` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `trip_data`
--

DROP TABLE IF EXISTS `trip_data`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `trip_data` (
  `id` int NOT NULL AUTO_INCREMENT,
  `trip_id` int NOT NULL,
  `position` varchar(255) NOT NULL,
  `time_stamp` datetime DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`id`),
  KEY `trip_id` (`trip_id`),
  CONSTRAINT `trip_data_ibfk_1` FOREIGN KEY (`trip_id`) REFERENCES `trips` (`id`) ON DELETE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=1121 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `trip_data`
--

LOCK TABLES `trip_data` WRITE;
/*!40000 ALTER TABLE `trip_data` DISABLE KEYS */;
/*!40000 ALTER TABLE `trip_data` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `trips`
--

DROP TABLE IF EXISTS `trips`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `trips` (
  `id` int NOT NULL AUTO_INCREMENT,
  `user_id` int NOT NULL,
  `start_location` varchar(255) DEFAULT NULL,
  `end_location` varchar(255) DEFAULT NULL,
  `price` int DEFAULT '0',
  `trip_date` datetime DEFAULT CURRENT_TIMESTAMP,
  `finished_trip` tinyint(1) DEFAULT '0',
  PRIMARY KEY (`id`),
  KEY `user_id` (`user_id`),
  CONSTRAINT `trips_ibfk_1` FOREIGN KEY (`user_id`) REFERENCES `users` (`id`) ON DELETE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=243 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `trips`
--

LOCK TABLES `trips` WRITE;
/*!40000 ALTER TABLE `trips` DISABLE KEYS */;
INSERT INTO `trips` VALUES (225,1,'Sluseholmen St.','Sluseholmen St.',17,'2024-12-14 23:12:28',1),(227,1,'Amagerbro St.','Sluseholmen St.',17,'2024-12-15 09:56:11',1),(228,1,'Sluseholmen St.','Sluseholmen St.',17,'2024-12-15 17:22:49',1),(229,1,'Sluseholmen St.','Sture St.',17,'2024-12-15 17:29:20',1),(230,1,'Sture St.','CPH H',36,'2024-12-15 17:46:26',1),(231,1,'CPH H','CPH H',17,'2024-12-15 18:04:27',1),(232,1,'CPH H','CPH H',17,'2024-12-15 18:18:13',1),(233,1,'CPH H','CPH H',17,'2024-12-15 18:22:18',1),(234,1,'CPH H','CPH H',17,'2024-12-15 18:25:45',1),(235,1,'CPH H','CPH H',17,'2024-12-15 18:28:50',1),(236,1,'CPH H','CPH H',17,'2024-12-15 18:31:42',1),(237,1,'CPH H','CPH H',17,'2024-12-15 18:33:52',1),(238,1,'CPH H','CPH H',17,'2024-12-15 18:36:04',1),(239,6,'CPH H','CPH H',17,'2024-12-15 18:37:07',1),(240,6,'CPH H','CPH H',17,'2024-12-15 18:40:34',1),(241,7,'CPH H','Sluseholmen St.',24,'2024-12-15 18:40:45',1),(242,6,'Sluseholmen St.','Sluseholmen St.',17,'2024-12-15 18:42:46',1);
/*!40000 ALTER TABLE `trips` ENABLE KEYS */;
UNLOCK TABLES;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = latin1 */ ;
/*!50003 SET character_set_results = latin1 */ ;
/*!50003 SET collation_connection  = latin1_swedish_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'ONLY_FULL_GROUP_BY,STRICT_TRANS_TABLES,NO_ZERO_IN_DATE,NO_ZERO_DATE,ERROR_FOR_DIVISION_BY_ZERO,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
/*!50003 CREATE*/ /*!50017 DEFINER=`root`@`localhost`*/ /*!50003 TRIGGER `update_finished_trip` BEFORE UPDATE ON `trips` FOR EACH ROW BEGIN
    IF NEW.end_location IS NOT NULL THEN
        SET NEW.finished_trip = TRUE;
    ELSE
        SET NEW.finished_trip = FALSE;
    END IF;
END */;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;

--
-- Table structure for table `users`
--

DROP TABLE IF EXISTS `users`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `users` (
  `id` int NOT NULL AUTO_INCREMENT,
  `username` varchar(255) NOT NULL,
  `password` varchar(255) NOT NULL,
  `is_checked_in` tinyint(1) NOT NULL DEFAULT '0',
  `in_transport` tinyint(1) DEFAULT '0',
  `in_transport_updated_at` datetime DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  `wallet` int DEFAULT '1500',
  PRIMARY KEY (`id`),
  UNIQUE KEY `username` (`username`)
) ENGINE=InnoDB AUTO_INCREMENT=8 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `users`
--

LOCK TABLES `users` WRITE;
/*!40000 ALTER TABLE `users` DISABLE KEYS */;
INSERT INTO `users` VALUES (1,'user','ebdf8cc00bc4d9ceee633c56c63b49955769a92ca060825c9b08e4af61326e2b',0,0,'2024-12-15 18:38:10',1213),(6,'qwe','4ac8d99fe7077956dfa6e23acf2ec791b1a28d53511561a56829ecef7227a249',0,0,'2024-12-15 18:44:50',1449),(7,'sander','ebdf8cc00bc4d9ceee633c56c63b49955769a92ca060825c9b08e4af61326e2b',0,0,'2024-12-15 18:43:31',1400);
/*!40000 ALTER TABLE `users` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `vehicles`
--

DROP TABLE IF EXISTS `vehicles`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `vehicles` (
  `id` int NOT NULL AUTO_INCREMENT,
  `vehicle_name` varchar(255) DEFAULT NULL,
  `current_location` varchar(255) DEFAULT NULL,
  `ammount_of_passengers` int DEFAULT '0',
  `registered_at` datetime DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=2 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `vehicles`
--

LOCK TABLES `vehicles` WRITE;
/*!40000 ALTER TABLE `vehicles` DISABLE KEYS */;
INSERT INTO `vehicles` VALUES (1,'Metro','Sluseholmen St.',0,'2024-12-12 20:14:42');
/*!40000 ALTER TABLE `vehicles` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2024-12-15 22:14:39




--CREATE TABLE IF NOT EXISTS users (
--    id INT AUTO_INCREMENT PRIMARY KEY,
--    username VARCHAR(255) NOT NULL UNIQUE,
--    password VARCHAR(255) NOT NULL
--);
--
--CREATE TABLE IF NOT EXISTS trips (
--    id INT AUTO_INCREMENT PRIMARY KEY,
--    user_id INT NOT NULL,
--    start_location VARCHAR(255) DEFAULT NULL,
--    end_location VARCHAR(255) DEFAULT NULL,
--    price INT DEFAULT 0,
--    trip_date DATETIME DEFAULT CURRENT_TIMESTAMP,
--    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
--);
--
--CREATE TABLE IF NOT EXISTS trip_data (
--    id INT AUTO_INCREMENT PRIMARY KEY,
--    trip_id INT NOT NULL,
--    position VARCHAR(255) NOT NULL,
--    time_stamp DATETIME DEFAULT CURRENT_TIMESTAMP,
--    FOREIGN KEY (trip_id) REFERENCES trips(id) ON DELETE CASCADE
--);
--
--CREATE TABLE IF NOT EXISTS vehicles (
--    id INT AUTO_INCREMENT PRIMARY KEY,
--    vehicle_name VARCHAR(255),
--    current_location VARCHAR(255),
--    ammount_of_passengers INT DEFAULT 0,
--    registered_at DATETIME DEFAULT CURRENT_TIMESTAMP
--);

--CREATE TABLE IF NOT EXISTS bluetooth_devices (
--    id INT AUTO_INCREMENT PRIMARY KEY,
--    vehicle_id INT NOT NULL,
--    device_name VARCHAR(255),
--    secret VARCHAR(255),
--    FOREIGN KEY (vehicle_id) REFERENCES vehicles(id) ON DELETE CASCADE
--);

--INSERT INTO vehicles (vehicle_name, current_location, ammount_of_passengers) VALUES ('Metro', 'Sluseholmen St.', 0);
--INSERT INTO bluetooth_devices (vehicle_id, device_name, secret) VALUES (1, 'ESP32-1111', 'very_SECRET_esp_1111');
--INSERT INTO bluetooth_devices (vehicle_id, device_name, secret) VALUES (1, 'ESP32-2222', 'very_SECRET_esp_2222');






