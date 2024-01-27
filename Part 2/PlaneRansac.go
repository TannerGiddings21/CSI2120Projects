//Tanner Giddings
//300172545
//CSI2120
//go run PlaneRansac.go PointCloud1.xyz 0.50 0.7 0.1

package main

import (
	"math"
    "fmt"
    "os"
    "strconv"
    "strings"
    "math/rand"
    "sync"
    "time"
)

//Data type representing a point's location on a 3D plane
type Point3D struct {
	X float64
	Y float64
	Z float64
}

//Data type representing a plane's equation on a 3D plane (Ax + By + Cz = D)
type Plane3D struct {
	A float64
	B float64
	C float64
	D float64
}

//Data type representing a plane (see Plane3D) with the size of the support for the plane
type Plane3DwSupport struct {
	Plane3D
	SupportSize int
}

//Method to check for errors
func check(e error) {
	if e != nil {
		panic(e)
	}
}

// reads an XYZ file and returns a slice of Point3D
// https://gobyexample.com/reading-files 
func ReadXYZ(filename string) []Point3D {
	var points []Point3D
	// https://www.tutorialspoint.com/how-to-read-a-file-into-a-string-in-golang
	read_file, err := os.ReadFile(filename)
	file_contents := string(read_file)
	check(err)
	file_split := strings.Split(file_contents, "\n")

	var current_point_string []string
	var new_X float64
	var new_Y float64
	var new_Z float64
	var err_X error
	var err_Y error
	var err_Z error

	for i := 1; i < len(file_split); i++ {
		current_point_string = strings.Split(file_split[i], "\t")
		if len(current_point_string) == 3 {
			new_X, err_X = strconv.ParseFloat(strings.Trim(current_point_string[0], "\r"), 64)
			check(err_X)
			new_Y, err_Y = strconv.ParseFloat(strings.Trim(current_point_string[1], "\r"), 64)
			check(err_Y)
			new_Z, err_Z = strconv.ParseFloat(strings.Trim(current_point_string[2], "\r"), 64)
			check(err_Z)
			points = append(points, Point3D{new_X, new_Y, new_Z})
		}
	}
	return points
}

// saves a slice of Point3D into an XYZ file
// https://gobyexample.com/writing-files
func SaveXYZ(filename string, points []Point3D) {
	new_file, err1 := os.Create(filename)
	check(err1)
	str := "x \t y \t z \n"
	for i := 0; i < len(points); i++ {
		// https://www.educative.io/answers/how-can-we-convert-a-float-into-a-string-in-golang
		str += fmt.Sprintf("%v", points[i].X) + "\t" + fmt.Sprintf("%v", points[i].Y) + "\t" + fmt.Sprintf("%v", points[i].Z) + "\n"
	} 
	_, err2 := new_file.WriteString(str)
	check(err2)
	fmt.Println(filename + " created successfully!")
}

// computes the distance between points p1 and p2
func (p1 *Point3D) GetDistance(p2 *Point3D) float64 {
	return math.Sqrt(math.Pow(p1.X - p2.X, 2) + math.Pow(p1.Y - p2.Y, 2) + math.Pow(p1.Z - p2.Z, 2))
}

// computes the plane defined by a set of 3 points
func GetPlane(points [3]Point3D) Plane3D {
	new_A := (points[0].Y - points[1].Y) * (points[1].Z - points[2].Z) - (points[1].Y - points[2].Y) * (points[0].Z - points[1].Z)
	new_B := (points[1].X - points[2].X) * (points[0].Z - points[1].Z) - (points[0].X - points[1].X) * (points[1].Z - points[2].Z)
	new_C := (points[0].X - points[1].X) * (points[1].Y - points[2].Y) - (points[0].Y - points[1].Y) * (points[1].X - points[2].X)
	new_D := - (new_A * points[0].X + new_B * points[0].Y + new_C * points[0].Z)
	return Plane3D{new_A, new_B, new_C, new_D}
}

// computes the number of required RANSAC iterations
func GetNumberOfIterations(confidence float64, percentageOfPointsOnPlane float64) int {
	return int(math.Log(1 - confidence) / math.Log(1 - math.Pow(percentageOfPointsOnPlane, 3)))
}

//Method for verifying whether or not a point is within a range of size eps from a plane
func InRange(plane Plane3D, point Point3D, eps float64) bool {
 	return (plane.A * point.X + plane.B * point.Y + plane.C * point.Z + plane.D) / math.Sqrt(plane.A * plane.A + plane.B * plane.B + plane.C * plane.C) <= eps
}

// computes the support of a plane in a set of points
func GetSupport(plane Plane3D, points []Point3D, eps float64) Plane3DwSupport {
 	var count int = 0
 	for i := 0; i < len(points); i++ {
 		if InRange(plane, points[i], eps) {
 			count++
 		}
 	}
 	return Plane3DwSupport{plane, count}
}

// extracts the points that supports the given plane
// and returns them as a slice of points
func GetSupportingPoints(plane Plane3DwSupport, points []Point3D, eps float64) []Point3D {
 	var supporting_points []Point3D
 	for i := 0; i < len(points); i++ {
 		if (plane.A * points[i].X + plane.B * points[i].Y + plane.C * points[i].Z + plane.D) / math.Sqrt(plane.A * plane.A + plane.B * plane.B + plane.C * plane.C) <= eps {
 			supporting_points = append(supporting_points, points[i])
 		}
 	}
 	return supporting_points
}

// creates a new slice of points in which all points
// belonging to the plane have been removed
func RemovePlane(plane Plane3D, points []Point3D, eps float64) []Point3D {
	var trimmed_points []Point3D
 	for i := 0; i < len(points); i++ {
 		if InRange(plane, points[i], eps) == false {
 			trimmed_points = append(trimmed_points, points[i])
 		}
 	}
 	return trimmed_points
}

//Returns the plane from a Plane3DwSupport
func convPlanes(planeWsupport Plane3DwSupport) Plane3D {
	return Plane3D{planeWsupport.A, planeWsupport.B, planeWsupport.C, planeWsupport.D}
}

//Generates random points from the point list
//Will be used as a goroutine
func RandomPointGenerator(stop chan bool, out chan Point3D, points []Point3D) {
	for {
		select {
			case <- stop:
				return
			case out <- points[rand.Intn(len(points))]:
		}
	}
}

//Generates triplets from the random points
//Will be used as a goroutine
func GeneratePointTriple(stop chan bool, out chan [3]Point3D, in chan Point3D) {
	var new_points [3]Point3D
	//count := 0
	for {
		new_points[0] = <- in
		new_points[1] = <- in
		new_points[2] = <- in
		select {
			case <- stop:
				return
			case out <- new_points:
		}
	}
}

/*
Takes in a triplets of points and outputs them until it has reached num_iterations,
then it stops the goroutines for RandomPointGenerator and GeneratePointTriple and it stops outputing point triplets.
*/
func TakeN(stop []chan bool, in chan [3]Point3D, out chan [3]Point3D, num_iterations int) {
	count := 0
	for {
		select {
			case out <- (<- in):
				count++
				if count >= num_iterations {
					for i := 0; i < len(stop); i++ {
						stop[i] <- true
					}
					return
				}
		}
	}
}


/*
	Generates a plane from 3 points and output the plane in a channel.
	Will be used as a goroutine
*/
func PlaneEstimator(stop chan bool, in chan [3]Point3D, out chan Plane3D) {
	var new_points [3]Point3D
	for {
		select {
			case <- stop:
				return
			case new_points = <- in:
				out <- GetPlane(new_points)
		}
	}
}

/*
	Finds the supporting points for a plane.
	Will be used as a goroutine.
*/
func FindSupportingPoints(stop chan bool, in chan Plane3D, out chan Plane3DwSupport, points []Point3D, eps float64) {
	var new_plane Plane3D
	for {
		select {
			case <- stop:
				return
			case new_plane = <- in:
				out <- GetSupport(new_plane, points, eps)
		}
	}
}

/*
	Fans In different channels and combines them back into one.
	Will be used as a goroutine.
*/
func FanIn(stop chan bool, in chan Plane3DwSupport, out chan Plane3DwSupport) {
	var new_support Plane3DwSupport
	for {
		select {
			case <- stop:
				return
			case new_support = <- in:
				out <- new_support
		}
	}
}

/*
	Compares planes in order to find the 3 most dominant planes.
	Will be used as a goroutine.
*/
func DominantPlaneIdentifier(stop chan bool, in chan Plane3DwSupport, best_support *Plane3DwSupport, wg *sync.WaitGroup) {
	var new_support Plane3DwSupport
	for {
		select {
			case <- stop:
				return
			case new_support = <- in:
				if new_support.SupportSize > best_support.SupportSize {
					*best_support = new_support
				}
				wg.Done()
		}
	}
}

func main() {
	start := time.Now()
	filename := strings.Trim(os.Args[1], " ")
	confidence, _ := strconv.ParseFloat(strings.Trim(os.Args[2], " "), 64)
	percentage, _ := strconv.ParseFloat(strings.Trim(os.Args[3], " "), 64)
	eps, _ := strconv.ParseFloat(strings.Trim(os.Args[4], " "), 64)

	//Initialises dominant planes as trivial cases
	var best_support Plane3DwSupport

	//Channels in order to stop goroutines
	stop1 := make([]chan bool, 2)
	stop2 := make(chan bool)
	stop3 := make(chan bool)
	stop4 := make(chan bool)

	stop5 := make(chan bool)

	//The commented parts here are to show how it would look with 50 concurrent channels
	/*
	stop6 := make(chan bool)
	stop7 := make(chan bool)
	stop8 := make(chan bool)
	stop9 := make(chan bool)
	stop10 := make(chan bool)
	stop11 := make(chan bool)
	stop12 := make(chan bool)
	stop13 := make(chan bool)
	stop14 := make(chan bool)
	stop15 := make(chan bool)
	stop16 := make(chan bool)
	stop17 := make(chan bool)
	stop18 := make(chan bool)
	stop19 := make(chan bool)
	stop20 := make(chan bool)
	stop21 := make(chan bool)
	stop22 := make(chan bool)
	stop23 := make(chan bool)
	stop24 := make(chan bool)
	stop25 := make(chan bool)
	stop26 := make(chan bool)
	stop27 := make(chan bool)
	stop28 := make(chan bool)
	stop29 := make(chan bool)
	stop30 := make(chan bool)
	stop31 := make(chan bool)
	stop32 := make(chan bool)
	stop33 := make(chan bool)
	stop34 := make(chan bool)
	stop35 := make(chan bool)
	stop36 := make(chan bool)
	stop37 := make(chan bool)
	stop38 := make(chan bool)
	stop39 := make(chan bool)
	stop40 := make(chan bool)
	stop41 := make(chan bool)
	stop42 := make(chan bool)
	stop43 := make(chan bool)
	stop44 := make(chan bool)
	stop45 := make(chan bool)
	stop46 := make(chan bool)
	stop47 := make(chan bool)
	stop48 := make(chan bool)
	stop49 := make(chan bool)
	stop50 := make(chan bool)
	stop51 := make(chan bool)
	stop52 := make(chan bool)
	stop53 := make(chan bool)
	stop54 := make(chan bool)
	*/

	//Channels in order to synchronize goroutines
	randomPointChan := make(chan Point3D)
	pointTripletChan := make(chan [3]Point3D)
	trimmedTripletChan := make(chan [3]Point3D)
	planeChan := make(chan Plane3D)
	planeWsupportChan := make(chan Plane3DwSupport)
	planeWsupportFannedInChan := make(chan Plane3DwSupport)

	num_iterations := GetNumberOfIterations(confidence, percentage)
	points := ReadXYZ(filename)
	var wg sync.WaitGroup
	for i := 0; i < 3; i++ {
		best_support = Plane3DwSupport{Plane3D{0,0,0,0}, 0}
		if len(points) > 0 {
			wg.Add(num_iterations)
			
			//The following rows represent the concurrency pipeline for the RANSAC algorithm
			go RandomPointGenerator(stop1[0], randomPointChan, points)
			go GeneratePointTriple(stop1[1], pointTripletChan, randomPointChan)
			go TakeN(stop1, pointTripletChan, trimmedTripletChan, num_iterations)
			go PlaneEstimator(stop2, trimmedTripletChan, planeChan)
			
			go FindSupportingPoints(stop5, planeChan, planeWsupportChan, points, eps)
			//The commented parts here are to show how it would look with 50 concurrent channels
			/*
			go FindSupportingPoints(stop6, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop7, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop8, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop9, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop10, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop11, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop12, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop13, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop14, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop15, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop16, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop17, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop18, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop19, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop20, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop21, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop22, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop23, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop24, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop25, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop26, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop27, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop28, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop29, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop30, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop31, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop32, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop33, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop34, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop35, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop36, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop37, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop38, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop39, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop40, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop41, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop42, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop43, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop44, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop45, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop46, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop47, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop48, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop49, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop50, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop51, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop52, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop53, planeChan, planeWsupportChan, points, eps)
			go FindSupportingPoints(stop54, planeChan, planeWsupportChan, points, eps)
			*/
			
			go FanIn(stop3, planeWsupportChan, planeWsupportFannedInChan)
			go DominantPlaneIdentifier(stop4, planeWsupportFannedInChan, &best_support, &wg)

			//Waits until the last plane is compared then sends a message to stop the goroutines through the stop channels
			wg.Wait()
			stop2 <- true
			stop3 <- true
			stop4 <- true

			stop5 <- true
			//The commented parts here are to show how it would look with 50 concurrent channels
			/*
			stop6 <- true
			stop7 <- true
			stop8 <- true
			stop9 <- true
			stop10 <- true
			stop11 <- true
			stop12 <- true
			stop13 <- true
			stop14 <- true
			stop15 <- true
			stop16 <- true
			stop17 <- true
			stop18 <- true
			stop19 <- true
			stop20 <- true
			stop21 <- true
			stop22 <- true
			stop23 <- true
			stop24 <- true
			stop25 <- true
			stop26 <- true
			stop27 <- true
			stop28 <- true
			stop29 <- true
			stop30 <- true
			stop31 <- true
			stop32 <- true
			stop33 <- true
			stop34 <- true
			stop35 <- true
			stop36 <- true
			stop37 <- true
			stop38 <- true
			stop39 <- true
			stop40 <- true
			stop41 <- true
			stop42 <- true
			stop43 <- true
			stop44 <- true
			stop45 <- true
			stop46 <- true
			stop47 <- true
			stop48 <- true
			stop49 <- true
			stop50 <- true
			stop51 <- true
			stop52 <- true
			stop53 <- true
			stop54 <- true
			*/
		}
		//Generates files for each dominant plane, with all the points within their range
		supporting_points := GetSupportingPoints(best_support, points, eps)
		trimmed_filename := strings.Trim(filename, ".xyz")
		SaveXYZ(trimmed_filename + "_" + strconv.Itoa(i) + ".xyz", supporting_points)
		points = RemovePlane(convPlanes(best_support), points, eps)
	}
	timeElapsed := time.Since(start)
	fmt.Println(timeElapsed)
}