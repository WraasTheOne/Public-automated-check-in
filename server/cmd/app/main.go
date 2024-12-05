package main

import (
	"Public-automated-check-in/server/pkg/util"
)

func main() {

	const portnumber string = "SERVER_PORT"
	port := util.LoadEnv(portnumber)

	listener := util.StartServer(port)

	defer listener.Close()

}
