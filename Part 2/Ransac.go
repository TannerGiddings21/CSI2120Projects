//Tanner Giddings
//300172545
//CSI2120
//go run Ransac.go PointCloud1.xyz 0.05 0.07 1

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
 	fmt.Println(len(supporting_points))
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
/*
func convPlanes(planeWsupport Plane3DwSupport) Plane3D {
	return Plane3D{planeWsupport.A, planeWsupport.B, planeWsupport.C, planeWsupport.D}
}
*/

//Ajusts the values for the best, second best and third best Plane3D by Support value
func AjustPlanes(best_supports *[3]Plane3DwSupport, new_plane Plane3DwSupport) {
	if new_plane.SupportSize > best_supports[0].SupportSize {
		best_supports[2] = best_supports[1]
		best_supports[1] = best_supports[0]
		best_supports[0] = new_plane
	} else if new_plane.SupportSize > best_supports[1].SupportSize {
		best_supports[2] = best_supports[1]
		best_supports[1] = new_plane
	} else if new_plane.SupportSize > best_supports[2].SupportSize {
		best_supports[2] = new_plane
	}
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
func DominantPlaneIdentifier(stop chan bool, in chan Plane3DwSupport, best_supports *[3]Plane3DwSupport, wg *sync.WaitGroup) {
	var new_support Plane3DwSupport
	for {
		select {
			case <- stop:
				return
			case new_support = <- in:
				if new_support.SupportSize > best_supports[0].SupportSize {
					best_supports[2] = best_supports[1]
					best_supports[1] = best_supports[0]
					best_supports[0] = new_support
				} else if new_support.SupportSize > best_supports[1].SupportSize {
					best_supports[2] = best_supports[1]
					best_supports[1] = new_support
				} else if new_support.SupportSize > best_supports[2].SupportSize {
					best_supports[2] = new_support
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
	var best_support [3]Plane3DwSupport
	best_support[0] = Plane3DwSupport{Plane3D{0,0,0,0}, 0}
	best_support[1] = Plane3DwSupport{Plane3D{0,0,0,0}, 0}
	best_support[2] = Plane3DwSupport{Plane3D{0,0,0,0}, 0}

	num_iterations := GetNumberOfIterations(confidence, percentage)
	points := ReadXYZ(filename)
	var wg sync.WaitGroup
	wg.Add(num_iterations)

	//Channels in order to stop goroutines
	stop1 := make([]chan bool, 2)
	stop2 := make(chan bool)
	stop3 := make(chan bool)
	stop4 := make(chan bool)
	//stop5 := make(chan bool)
	//stop6 := make(chan bool)
	//stop7 := make(chan bool)
	stop8 := make(chan bool)
	stop9 := make(chan bool)

	//Channels in order to synchronize goroutines
	randomPointChan := make(chan Point3D)
	pointTripletChan := make(chan [3]Point3D)
	trimmedTripletChan := make(chan [3]Point3D)
	planeChan := make(chan Plane3D)
	planeWsupportChan := make(chan Plane3DwSupport)
	planeWsupportFannedInChan := make(chan Plane3DwSupport)
	
	//The following rows represent the concurrency pipeline for the RANSAC algorithm
	go RandomPointGenerator(stop1[0], randomPointChan, points)
	go GeneratePointTriple(stop1[1], pointTripletChan, randomPointChan)
	go TakeN(stop1, pointTripletChan, trimmedTripletChan, num_iterations)
	go PlaneEstimator(stop2, trimmedTripletChan, planeChan)
	go FindSupportingPoints(stop3, planeChan, planeWsupportChan, points, eps)
	go FindSupportingPoints(stop4, planeChan, planeWsupportChan, points, eps)
	go FanIn(stop8, planeWsupportChan, planeWsupportFannedInChan)
	go DominantPlaneIdentifier(stop9, planeWsupportFannedInChan, &best_support, &wg)

	wg.Wait()
	stop2 <- true
	stop3 <- true
	stop4 <- true
	//stop5 <- true
	//stop6 <- true
	//stop7 <- true
	stop8 <- true
	stop9 <- true

	//Generates files for each dominant plane, with all the points within their range
	trimmed_filename := strings.Trim(filename, ".xyz")
	SaveXYZ(trimmed_filename + "_1.xyz", GetSupportingPoints(best_support[0], points, eps))
	SaveXYZ(trimmed_filename + "_2.xyz", GetSupportingPoints(best_support[1], points, eps))
	SaveXYZ(trimmed_filename + "_3.xyz", GetSupportingPoints(best_support[2], points, eps))
	timeElapsed := time.Since(start)
	fmt.Println(timeElapsed)
}
