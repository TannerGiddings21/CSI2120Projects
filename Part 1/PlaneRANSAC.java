/** PlaneRANSAC class
 * @author Tanner Giddings 300172545
 */ 

import java.util.*;
import java.lang.Math;
import java.lang.NullPointerException;
import java.lang.NumberFormatException;

public class PlaneRANSAC {
	PointCloud pc;
	double eps;

	/** Constructor for PlaneRANSAC
	 * @param pc PointCloud on which operations will be done
	 */ 
	public PlaneRANSAC(PointCloud pc) {
		this.pc = pc;
	}

	/** Setter for eps variable
	 * @param eps value to assign to eps
	 */ 
	public void setEps(double eps) {
		this.eps = eps;
	}

	/** Getter for eps variable
	 * @return eps value
	 */ 
	public double getEps() {
		return eps;
	}

	/** Method to calculate the number of iterations to do in the run method
	 * @param confidence confidence threshold to apply to test
	 * @param percentageOfPointsOnPlane percentage of points on the plane
	 * @return number of iterations to do in the 'run' method
	 */ 
	public int getNumberOfIterations(double confidence, double percentageOfPointsOnPlane) {
		return (int)(Math.log(1 - confidence) / Math.log(1 - Math.pow(percentageOfPointsOnPlane, 3)));
	}

	/** Method to verify if a point is within an epsilon value of the plane
	 * @param pt Point to verify if it is in plane
	 * @param plane Plane to verify with
	 * @return boolean is or isn't in this range
	 */
	public boolean contained_eps(Point3D pt, Plane3D plane) {
		double dist = plane.getDistance(pt);
		return dist <= eps;
	} 

	/** Method to count all the points contained in an epsilon range from the plane
	 * @param plane plane from which to calculate the distances
	 * @return count of points within this interval
	 */
	public int count_points_in_plane(Plane3D plane) {
		int count = 0;
		Point3D current_point;
		Iterator<Point3D> new_point_list = pc.iterator();
		while (new_point_list.hasNext()) {
			current_point = new_point_list.next();
			if (contained_eps(current_point, plane)) {
				count++;
			}
		}
		return count;
	}

	/**Method to shift best_planes depending on where the current_plane goes
	 * @param best_planes array of best planes
	 * @param best_support array of supports of best planes
	 * @param current_plane plane to be added to array of best planes
	 * @param current_support support of current plane
	 * @param index index to add current_plane to best_planes
	 */ 
	public void shift_planes(Plane3D[] best_planes, int[] best_support, Plane3D current_plane, int current_support, int index) {
		for (int i = 2 - index; i > 0; i--) {
			best_planes[i] = best_planes[i-1];
			best_support[i] = best_support[i-1];
		}
		best_planes[index] = current_plane;
		best_support[index] = current_support;
	}

	/**Method to compare a new plane to the three current best planes and replace them if needed
	 * @param best_planes array of best planes
	 * @param best_support array of best supports for the best planes
	 * @param current_plane plane to compare to best planes
	 * @param current_support support of current plane
	 */ 
	public void compare_planes(Plane3D[] best_plane, int[] best_support, Plane3D current_plane, int current_support) {
		if (best_plane[0] == null) {
				best_plane[0] = current_plane;
				best_support[0] = current_support;
			} else if (best_plane[1] == null) {
				best_plane[1] = current_plane;
				best_support[0] = current_support;
			} else if (best_plane[2] == null) {
				best_plane[2] = current_plane;
				best_support[2] = current_support;
			} else {
				for (int i = 0; i < 3; i++) {
					if (best_support[i] < current_support) {
						shift_planes(best_plane, best_support, current_plane, current_support, i);
						break;
					}
				}
			}
	}

	/** Method which runs the RANSAC algorithm and saves an .xyz file with the 
	 * @param numberOfIterations number of iterations to be done in the algorithm
	 * @param filename name of the new file to be created with only the points in the dominant plane.
	 */
	public void run(int numberOfIterations, String filename) {
		/*Point3D p1; Point3D p2; Point3D p3; Point3D current_point;
		Plane3D best_plane = new Plane3D(); Plane3D current_plane;
		ArrayList<Point3D> other_plane; Iterator<Point3D> current_point_list = pc.iterator();
		int best_support; int current_support; int i; int j;
		boolean isFirst;
		for (j = 0; j < 3; j++) {
			best_support = 0;
			isFirst = true;
			for (i = 0; i < numberOfIterations; i++) {
				p1 = pc.getPoint();
				p2 = pc.getPoint();
				p3 = pc.getPoint();
				current_plane = new Plane3D(p1, p2, p3);
				if (!isFirst) {
					current_support = count_points_in_plane(current_plane);
					if (current_support > best_support) {
						best_plane = current_plane;
						best_support = current_support;
					}
				} else {
					best_plane = current_plane;
					best_support = count_points_in_plane(best_plane);
					isFirst = false;
				}
			}
			remove_points(current_point_list, j, filename, best_plane);
		}*/
		Point3D p1; Point3D p2; Point3D p3; Point3D current_point;
		Plane3D[] best_planes = new Plane3D[3]; Plane3D current_plane;
		int[] best_support = new int[3]; int current_support; int i;
		for (i = 0; i < numberOfIterations; i++) {
			p1 = pc.getPoint();
			p2 = pc.getPoint();
			p3 = pc.getPoint();
			current_plane = new Plane3D(p1, p2, p3);
			current_support = count_points_in_plane(current_plane);
			compare_planes(best_planes, best_support, current_plane, current_support);
		}
		for (i = 0; i < 3; i++) {
			remove_points(i, filename, best_planes[i]);
		}
	}

	/**Method to remove the .xyz at the end of a filename when creating a new filename
	 * @param filename filename where .xyz needs to be removed
	 * @return filename without .xyz
	 */
	public String remove_xyz(String filename) {
		String new_filename = "";
		for (int i = 0; i < filename.length() - 4; i++) {
			new_filename += filename.charAt(i);
		}
		return new_filename;
	}

	/**Method to generate a new filename
	 * @param filename filename to be inspired from
	 * @param index number of the file
	 * @return new file name
	 */
	public String generate_new_filename(String filename, int index) {
		return remove_xyz(filename) + index + ".xyz";
	}

	/** Method to remove points from dominant plane from the point cloud and creates a new .xyz file with those points
	 * @param current_point_list ArrayList of points
	 * @param j file number
	 * @param filename name to save file
	 * @param best_plane best plane for iteration for which to remove points
	 */ 
	public void remove_points(int j, String filename, Plane3D best_plane) {
		PointCloud dom_plane = new PointCloud();
		Point3D current_point;
		Iterator<Point3D> new_point_list = pc.iterator();
		while (new_point_list.hasNext()) {
			current_point = new_point_list.next();
			if (contained_eps(current_point, best_plane)) {
				dom_plane.addPoint(current_point);
			}
		}
		dom_plane.save(generate_new_filename(filename, j));
		System.out.println("There are " + dom_plane.getNumPoints() + " points in this pointCloud \n");
	}

	/** Main method
	 * @param args file name to be entered and used for point clouds
	 */
	public static void main(String[] args) {
		double epsilon; String input; double confidence = 0.1; double percentageOfPointsOnPlane = 0.1;
		boolean isCorrect = false;
		Scanner sc = new Scanner(System.in);
		String filename = args[0].trim();
		PointCloud pc = new PointCloud(filename);
		PlaneRANSAC pRansac = new PlaneRANSAC(pc);
		pRansac.setEps(1);
		int numberOfIterations = pRansac.getNumberOfIterations(0.5, 0.1);
		pRansac.run(numberOfIterations, filename);
	}
}