/** Vector class
 * @author Tanner Giddings 300172545
 */ 

public class Vector {
	private double x;
	private double y;
	private double z;

	/**Builds a vector between two points
	 * Formula from MAT1741 Course - Summer 2021, Instructor: Joseph Khoury
	 * @param a first point to build vector
	 * @param b second point to build vector
	 */ 
	public Vector(Point3D a, Point3D b) {
		x = b.getX() - a.getX();
		y = b.getY() - a.getY();
		z = b.getZ() - a.getZ();
	}

	/** Constructor for vector using all instance variables
	 * @param x x coordinate of vector
	 * @param y y coordinate of vector
	 * @param z z coordinate of vector
	 */ 
	public Vector(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	/** Empty constructor with no parameters.
	 */ 
	public Vector() {}

	/** Getter for variable x
	 * @return x coordinate of vector
	 */ 
	public double getX() {
		return x;
	}

	/** Setter for variable x
	 * @param x x coordinate of vector
	 */ 
	public void setX(double x) {
		this.x = x;
	}

	/** Getter for variable y
	 * @return y coordinate of vector
	 */ 
	public double getY() {
		return y;
	}

	/** Setter for variable y
	 * @param y y coordinate of vector
	 */ 
	public void setY(double y) {
		this.y = y;
	}

	/** Getter for variable z
	 * @return z coordinate of vector
	 */ 
	public double getZ() {
		return z;
	}

	/** Setter for variable z
	 * @param z z coordinate of vector
	 */ 
	public void setZ(double z) {
		this.z = z;
	}

	/** Method to get normal vector of this vector and another one
	 * Formula from MAT1741 Course - Summer 2021, Instructor: Joseph Khoury
	 * @param vector other vector from which to get the normal vector
	 * @return normal vector
	 */ 
	public Vector getNormalVector(Vector vector) {
		Vector normal_vector = new Vector();
		normal_vector.setX(y * vector.getZ() - vector.getY() * z);
		normal_vector.setY(vector.getX() * z - x * vector.getZ());
		normal_vector.setZ(x * vector.getY() - y * vector.getX());
		return normal_vector;
	}

}