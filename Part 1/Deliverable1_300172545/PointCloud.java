/** PointCloud class
 * @author Tanner Giddings 200172545
 */ 

import java.util.*;
import java.io.File;
import java.io.IOException;
import java.io.FileNotFoundException;
import java.io.FileWriter;

public class PointCloud{
	ArrayList<Point3D> pointList;

	/** Constructor for PointCloud class
	 * @param filename name of the file to be used for the point cloud
	 */
	public PointCloud(String filename) {
		//https://www.w3schools.com/java/java_files_read.asp
		//https://stackoverflow.com/questions/52431907/java-read-x-y-z-coordinates-from-a-file
		pointList = new ArrayList<Point3D>();
		try {
			File myObj = new File(filename);
			boolean isFirst = true;
			Scanner myReader = new Scanner(myObj);
			while (myReader.hasNextLine()) {
				String[] data = myReader.nextLine().split("\t");
				if (!isFirst) {
					pointList.add(new Point3D(Double.parseDouble(data[0]), Double.parseDouble(data[1]), Double.parseDouble(data[2])));
				} else {
					isFirst = false;
				}
			}
			myReader.close();
		} catch (FileNotFoundException e) {
			System.out.println("An error occured.");
			e.printStackTrace();
		}
		System.out.println("There are " + pointList.size() + " points in this point cloud.");
	}

	/**Empty constructor for PointCloud. Initialises an arraylist.
	 */
	public PointCloud() {
		pointList = new ArrayList<Point3D>();
	}

	/** Method to set the points in the point cloud after removal
	 * @param pointList new list of points
	 */
	public void setPointList(ArrayList<Point3D> pointList) {
		this.pointList = pointList;
	} 

	/** Method to add a point to the point cloud
	 * @param pt point to be added
	 */ 
	public void addPoint(Point3D pt) {
		pointList.add(pt);
	}

	/** Gets a random point from the point cloud
	 * @return random point
	 */ 
	public Point3D getPoint() {
		if (pointList.size() > 1) {
			return pointList.get((int)(Math.floor(Math.random() * pointList.size())));
		} else if (pointList.size() == 1) {
			return pointList.get(1);
		} else {
			return new Point3D();
		}
	}

	/** Saves the point cloud as a new file
	 * @param filename name of the new file to be created
	 */ 
	public void save(String filename) {
		//https://www.w3schools.com/java/java_files_create.asp
		try {
			File new_file = new File(filename);
			if (new_file.createNewFile()) {
				System.out.println("File created: " + new_file.getName());
				writeToFile(filename);
			} else {
				System.out.println("File already exists");
			}
		} catch (IOException e) {
			System.out.println("An error occured");
			e.printStackTrace();
		}
	}

	/** Method to write the contents of the point list a file
	 * @param filename name of the file to be written to
	 */ 
	public void writeToFile(String filename) {
		//https://www.w3schools.com/java/java_files_create.asp
		try {
			FileWriter myWriter = new FileWriter(filename);
			Point3D next;
			myWriter.write("x" + "\t" + "y" + "\t" + "z" + "\n");
			for (int i = 0; i < pointList.size(); i++) {
				myWriter.write(pointList.get(i).toString() + "\n");
			}
			myWriter.close();
			System.out.println("Sucessfully wrote to file " + filename);
		} catch (IOException e) {
			System.out.println("Error occured");
		}
	}

	/** Returns the data structure used to represent the point cloud
	 * @return ArrayList<Point3D> representing the point cloud
	 */ 
	public Iterator<Point3D> iterator() {
		return pointList.iterator();
	}

	/** Returns number of points in point cloud
	 * @return number of points in pointcloud
	 */
	 public int getNumPoints() {
	 	return pointList.size();
	 } 
}