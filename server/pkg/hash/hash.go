package hash

import (
	"crypto/sha256"
	"fmt"
)

func HashPassword(password string) string {
	// Hash the password using SHA-256 algorithm
	hash := fmt.Sprintf("%x", sha256.Sum256([]byte(password)))

	return string(hash[:])
}

func ComparePasswords(hashedPassword, password string) bool {
	// Compare the hashed password with the password provided by the user

	return hashedPassword == HashPassword(password)
}
