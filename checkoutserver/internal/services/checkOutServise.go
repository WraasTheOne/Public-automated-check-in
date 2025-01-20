package checkinServise

import (

	"Public-automated-check-in/checkinserver/internal/db"
	"Public-automated-check-in/checkinserver/pkg/util"
	"fmt"
)

// make grpc call for auth service using roken from incomming request
func CheckinService() {
	// get db envs
	var usernamedb, passworddb, hostdb, portdb, dbname = util.GetDbEnvs()
	if err := db.InitDB(usernamedb, passworddb, hostdb, portdb, dbname); err != nil {
		fmt.Println("Failed to connect to database")
		return
	}
	defer db.CloseDB()

	// get user id to update
	id := db.GetFinishedUsers()
	if len(id) == 0 {
		return
	}

	//for each user id update the user
	for _, id := range id {
		// get last location
		tripID, err := db.GetAndSetLastLocation(id)
		if err != nil {
			fmt.Println("Failed to get last location")
			return
		}
		// get all locations the user has been
		locations, err := db.GetLocations(id, tripID)
		if err != nil {
			fmt.Println("Failed to get locations")
			return
		}
		// get how different the zones are from the locations thre can be multiple zones
		price := util.GetPrice(locations)
		fmt.Println("Price: ", price, " kr")

		err = db.UpdateTripPrice(tripID, price)
		if err != nil {
			fmt.Println("Failed to update trip price")
			return
		}
		//update user wallet
		if err := db.UpdateUserWallet(id, price); err != nil {
			fmt.Println("Failed to update user wallet")
			return
		}
		fmt.Println("User wallet updated")

	}

}

