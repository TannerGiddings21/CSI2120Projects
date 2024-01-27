/** Point3D class
 * @author Tanner Giddings 300172545
 */ 

import java.lang.Math;

public class Point3D {
	private double x; //Coordinate on the x plane
	private double y; //Coordinate on the y plane
	private double z; //Coordinate on the z plane

	/** Constructor for the Point3D class using the three instance variables, x, y and z.
	 * @param x represents location of point along x plane, double.
	 * @param y represents location of point along y plane, double.
	 * @param z represents location of point along z plane, double.
	 */ 
	public Point3D(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	/** Constructor for Point3D class with no inputs.
	 */ 
	public Point3D() {}

	/** Getter method for x instance variable
	 * @return double representing location of point along x plane.
	 */ 
	public double getX() {
		return x;
	}

	/** Setter method for x instance variable
	 * @param x location of point on x plane.
	 */ 
	public void setX(double x) {
		this.x = x;
	}

	/** Getter method for y instance variable
	 * @return double representing location of point along y plane.
	 */ 
	public double getY() {
		return y;
	}

	/** Setter method for x instance variable
	 * @param y location of point on x plane.
	 */ 
	public void setY(double y) {
		this.y = y;
	}

	/** Getter method for z instance variable
	 * @return double representing location of point along z plane.
	 */ 
	public double getZ() {
		return z;
	}

	/** Setter method for x instance variable
	 * @param z location of point on x plane.
	 */ 
	public void setZ(double z) {
		this.z = z;
	}

	/** Method to return values of Point3D class into a string
	 * @return String of values of point
	 */ 
	public String toString() {
		return x + "\t" + y + "\t" + z;
	}
}