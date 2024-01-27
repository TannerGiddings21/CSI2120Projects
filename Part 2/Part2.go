//Tanner Giddings
//300172545
//CSI2120
//go run Part2.go PointCloud1.xyz 0.05 0.07 1


package main

import (
	"math"
    "fmt"
    "os"
    "strconv"
    "strings"
    "math/rand"
    "sync"
)

type Point3D struct {
	X float64
	Y float64
	Z float64
}

type Plane3D struct {
	A float64
	B float64
	C float64
	D float64
}

type Plane3DwSupport struct {
	Plane3D
	SupportSize int
}

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
func GetPlane(points []Point3D) Plane3D {
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
func GetSupportingPoints(plane Plane3D, points []Point3D, eps float64) []Point3D {
 	var supporting_points []Point3D
 	for i := 0; i < len(points); i++ {
 		if InRange(plane, points[i], eps) {
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

func convPlanes(planeWsupport Plane3DwSupport) Plane3D {
	return Plane3D{planeWsupport.A, planeWsupport.B, planeWsupport.C, planeWsupport.D}
}

func AjustPlanes(best_support1 Plane3DwSupport, best_support2 Plane3DwSupport, best_support3 Plane3DwSupport, new_plane Plane3DwSupport) {
	if new_plane.SupportSize > best_support1.SupportSize {
		best_support3 = best_support2
		best_support2 = best_support1
		best_support1 = new_plane
	} else if new_plane.SupportSize > best_support2.SupportSize {
		best_support3 = best_support2
		best_support2 = new_plane
	} else if new_plane.SupportSize > best_support3.SupportSize {
		best_support3 = new_plane
	}
}

/*
	ADDING METHODS TO SYNCHRONIZE CHANNELS AND STUFF!!!
*/
//https://gobyexample.com/random-numbers
//https://go.dev/blog/pipelines
/*
func generatePoints(point_triples chan []Point3D, points []Point3D, num_iterations int) <- chan []Point3D {
	go func() {
		for i := 0; i < num_iterations; i++ {
			new_points := make([]Point3D, 3)
			new_points[0] = points[rand.Intn(len(points))]
			new_points[1] = points[rand.Intn(len(points))]
			new_points[2] = points[rand.Intn(len(points))]
			point_triples <- new_points
			fmt.Println(i)
		}
	}()
	return point_triples
}
*/
func generatePoints(points []Point3D, wg1 *sync.WaitGroup, stop <- chan bool) {
	point_triples = make(chan []Point3D)
	go func() {
		defer func() {wg1.Done()}()
		defer close(point_triples)
		for {
			new_points := make([]Point3D, 3)
			new_points[0] = points[rand.Intn(len(points))]
			new_points[1] = points[rand.Intn(len(points))]
			new_points[2] = points[rand.Intn(len(points))]
			point_triples <- new_points
			fmt.Println(i)
			select:
				case <- stop:
					return
				case point_triples <- new_points:
		}
	}()
}

func makePlaneWChan(new_plane chan Plane3D, in <- chan []Point3D) {
	go func() {
		for i := range in {
			new_plane <- GetPlane(i)
		}
	}()
}

func GetSupportWChan(new_planeWsupport chan Plane3DwSupport, in <- chan Plane3D, points []Point3D, eps float64) {
	go func() {
		for i := range in {
			new_planeWsupport <- GetSupport(i, points, eps)
		}
	}()
}

func findBestSupports(in <- chan Plane3DwSupport, best_support1 Plane3DwSupport, best_support2 Plane3DwSupport, best_support3 Plane3DwSupport, wg *sync.WaitGroup) {
	go func() {
		defer func() {wg.Done()}()
		for current := range in {
			AjustPlanes(best_support1, best_support2, best_support3, current)
		}
	}()
}


// STILL NOT ADDING POINTS
// It can compile though...
func main() {
	// https://www.educative.io/answers/what-are-command-line-arguments-in-golang
	// https://www.geeksforgeeks.org/how-to-trim-a-string-in-golang/
	filename := strings.Trim(os.Args[1], " ")
	confidence, _ := strconv.ParseFloat(strings.Trim(os.Args[2], " "), 64)
	percentage, _ := strconv.ParseFloat(strings.Trim(os.Args[3], " "), 64)
	eps, _ := strconv.ParseFloat(strings.Trim(os.Args[4], " "), 64)

	best_support1 := Plane3DwSupport{Plane3D{0,0,0,0}, 0}
	best_support2 := Plane3DwSupport{Plane3D{0,0,0,0}, 0}
	best_support3 := Plane3DwSupport{Plane3D{0,0,0,0}, 0}
	num_iterations := GetNumberOfIterations(confidence, percentage)
	var wg sync.WaitGroup
	wg.Add(num_iterations )
	fmt.Println(num_iterations)
	points := ReadXYZ(filename)

	point_triples := make(chan []Point3D)
	new_plane := make(chan Plane3D)
	new_planeWsupport := make(chan Plane3DwSupport)

	ch1 := generatePoints(point_triples, points, num_iterations)
	ch2 := makePlaneWChan(new_plane, ch1)
	ch3 := GetSupportWChan(new_planeWsupport, ch2, points, eps)
	findBestSupports(ch3, best_support1, best_support2, best_support3, &wg)

	//findBestSupports(GetSupportWChan(makePlaneWChan(generatePoints(points, num_iterations)), points, eps), best_support1, best_support2, best_support3, &wg)
	
	wg.Wait()
	close(point_triples)
	close(new_plane)
	close(new_planeWsupport)
	trimmed_filename := strings.Trim(filename, ".xyz")
	SaveXYZ(trimmed_filename + "_1.xyz", GetSupportingPoints(convPlanes(best_support1), points, eps))
	SaveXYZ(trimmed_filename + "_2.xyz", GetSupportingPoints(convPlanes(best_support2), points, eps))
	SaveXYZ(trimmed_filename + "_3.xyz", GetSupportingPoints(convPlanes(best_support3), points, eps))
}