package main

//go run Args.go PointCloud1.xyz 0.05 0.9 1

import (
	"os"
	"fmt"
	"strings"
)

func check(e error) {
	if e != nil {
		panic(e)
	}
}

func ReadFile(filename string) {

	read_file, err := os.ReadFile(filename)
	file_contents := string(read_file)
	check(err)
	file_split := strings.Split(file_contents, "\n")

	for i := 1; i < len(file_split); i++ {
		current_point_string := strings.Split(file_split[i], "\t")
		fmt.Println(current_point_string)
	}
}

func main() {
	/*
	toGetAllArgs := os.Args[1:]
	for i := 0; i < len(toGetAllArgs); i++ {
		fmt.Println(toGetAllArgs[i])
	}
	*/
	ReadFile(os.Args[1])
}