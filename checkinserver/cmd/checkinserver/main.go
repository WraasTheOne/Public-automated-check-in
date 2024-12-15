package main

import (
	"Public-automated-check-in/checkinserver/internal/services"
	"time"

)


func main() {

	for {

		checkinServise.CheckinService()
		time.Sleep(2 * time.Minute)
	}
}

