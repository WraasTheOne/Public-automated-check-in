package service

import (
	"context"
	"crypto/sha256"
	"database/sql"
	"fmt"
	"strings"

	"Public-automated-check-in/server/internal/db"
	"Public-automated-check-in/server/internal/redisdb"
	"Public-automated-check-in/server/pkg/hash"
	"Public-automated-check-in/server/pkg/util"
	"Public-automated-check-in/server/proto"
)

type LoginService struct {
	proto.UnimplementedLoginServiceServer
}

func (s *LoginService) Login(ctx context.Context, req *proto.LoginRequest) (*proto.LoginResponse, error) {
	// Load environment variables for database configuration
	var usernamedb, passworddb, hostdb, portdb, dbname = util.GetDbEnvs()

	// Retrieve incoming username and password from the request
	username := req.GetUsername()
	incpassword := req.GetPassword()

	if username == "" || incpassword == "" {
		return &proto.LoginResponse{
			Message: "Invalid username or password",
		}, nil
	}

	// Initialize database connection
	if err := db.InitDB(usernamedb, passworddb, hostdb, portdb, dbname); err != nil {
		fmt.Print(err)
		return nil, fmt.Errorf("failed to connect to database: %v", err)
	}
	// Fetch user details from the database using the provided username

	id, password, err := db.GetUser(username)
	if err != nil {
		if err == sql.ErrNoRows {
			return &proto.LoginResponse{
				Message: "Invalid username or password",
			}, nil
		}
		return nil, fmt.Errorf("failed to fetch user from database: %v", err)
	}

	defer db.CloseDB()

	if hash.ComparePasswords(password, incpassword) {
		return &proto.LoginResponse{
			Message: "Invalid username or password",
		}, nil
	}

	// Create a new session for the user if authentication is successful
	session, err := util.CreateSession(id)
	if err != nil {
		return nil, fmt.Errorf("failed to create session: %v", err)
	}

	redisdb.InitRedis()
	redisdb.SetWithExpiration(session.Token, id, 3600)
	fmt.Println("im here")
	defer redisdb.CloseRedis()

	// Return a successful response with the session token
	return &proto.LoginResponse{
		Message: "Login successful",
		Token:   session.Token,
	}, nil
}

func (s *LoginService) Register(ctx context.Context, req *proto.RegisterRequest) (*proto.RegisterResponse, error) {
	// Load environment variables for database configuration
	var usernamedb, passworddb, hostdb, portdb, dbname = util.GetDbEnvs()

	// Retrieve username and password from the request
	username := req.GetUsername()
	rawPassword := req.GetPassword()

	// Hash the password using SHA-256
	hashedPassword := sha256.Sum256([]byte(rawPassword))
	hashedPasswordStr := fmt.Sprintf("%x", hashedPassword)

	// Initialize database connection
	if err := db.InitDB(usernamedb, passworddb, hostdb, portdb, dbname); err != nil {
		return nil, fmt.Errorf("failed to connect to database: %v", err)
	}

	// Insert the user into the database
	userID, err := db.RegisterUser(username, hashedPasswordStr)
	if err != nil {
		if strings.Contains(err.Error(), "Duplicate entry") {
			return &proto.RegisterResponse{
				Message: "Username already exists",
			}, nil
		}
		return nil, fmt.Errorf("failed to register user: %v", err)
	}

	defer db.CloseDB() // Ensure the database connection is closed after the operation
	// Return a successful response with the new user's ID
	return &proto.RegisterResponse{
		UserId:  fmt.Sprintf("%d", userID),
		Message: "Registration successful",
	}, nil
}
