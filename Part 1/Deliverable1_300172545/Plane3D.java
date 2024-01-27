/** Plane3D class
 * @author Tanner Giddings 300172545
 */ 

import java.lang.Math;

public class Plane3D {
	//Instance variables representing a plane in the equation ax + by + cz = d.
	private double a; //Parameter for plane equation related to x plane
	private double b; //Parameter for plane equation related to y plane
	private double c; //Parameter for plane equation related to z plane
	private double d; //Parameter for plane equation with ax + by + cz = d

	/** Constructor from parameters a, b, c, d to construct plane in equation ax + by + cz = d.
	 * @param a double
	 * @param b double
	 * @param c double
	 * @param d double
	 */ 
	public Plane3D(double a, double b, double c, double d) {
		this.a = a;
		this.b = b;
		this.c = c;
		this.d = d;
	}

	/** Empty constructor for Plane2D
	 */
	public Plane3D() {} 

	/** Constructor from the points p1, p2, p3 to construct a plane following the equation ax + by + cz = d.
	 * Formula from MAT1741 Course - Summer 2021, Instructor: Joseph Khoury
	 * @param p1 Point3D 1 to construct plane
	 * @param p2 Point3D 2 to construct plane
	 * @param p3 Point3D 3 to construct plane
	 */ 
	public Plane3D(Point3D p1, Point3D p2, Point3D p3) {
		Vector v1 = new Vector(p1, p2);
		Vector v2 = new Vector(p2, p3);
		Vector normal_vector = v1.getNormalVector(v2);
		a = normal_vector.getX();
		b = normal_vector.getY();
		c = normal_vector.getZ();
		d = - (a * p1.getX() + b * p1.getY() + c * p1.getZ());
	}

	/** Method to find distance between the current plane and a point.
	 * Formula from MAT1741 Course - Summer 2021, Instructor: Joseph Khoury
	 * @param pt Point3D to find distance between
	 * @return distance between the plane and pt
	 */ 
	public double getDistance(Point3D pt) {
		return Math.abs((a * pt.getX() + b * pt.getY() + c * pt.getZ() + d) / Math.sqrt(a*a + b*b + c*c));
	}

	/** Method to get the equation for the plane
	 * @return String value of equation
	 */
	 public String toString() {
	 	return a + "x + " + b + "y + " + c + "z = " + d;
	 } 
}