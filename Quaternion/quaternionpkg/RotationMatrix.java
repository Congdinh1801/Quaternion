package quaternionpkg;

import java.math.BigInteger;

public class RotationMatrix {
	private double r11 = 0; // First row
	private double r12 = 0;
	private double r13 = 0;
	private double r21 = 0; // Second row
	private double r22 = 0;
	private double r23 = 0;
	private double r31 = 0; // Third row
	private double r32 = 0;
	private double r33 = 0;

	public RotationMatrix(double[] matrix) {
		this.r11 = matrix[0];
		this.r12 = matrix[1];
		this.r13 = matrix[2];
		this.r21 = matrix[3];
		this.r22 = matrix[4];
		this.r23 = matrix[5];
		this.r31 = matrix[6];
		this.r32 = matrix[7];
		this.r33 = matrix[8];

	}

	public RotationMatrix yawPitchRollToMatrix(double yaw, // z angle (radians)
			double pitch, // y angle (radians)
			double roll) // x angle (radians)
	{
		// Precompute sines and cosines of Euler angles
		double su = Math.sin(roll);
		double cu = Math.cos(roll);
		double sv = Math.sin(pitch);
		double cv = Math.cos(pitch);
		double sw = Math.sin(yaw);
		double cw = Math.cos(yaw);

		// Create and populate RotationMatrix
		double[] firstMatrix = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		RotationMatrix A = new RotationMatrix(firstMatrix);
		A.r11 = cv * cw;
		A.r12 = su * sv * cw - cu * sw;
		A.r13 = su * sw + cu * sv * cw;
		A.r21 = cv * sw;
		A.r22 = cu * cw + su * sv * sw;
		A.r23 = cu * sv * sw - su * cw;
		A.r31 = -sv;
		A.r32 = su * cv;
		A.r33 = cu * cv;
		return A;
	}

	// code 3
	public double[] MatrixToYawPitchRoll(RotationMatrix A) {
		double[] angles = new double[3];
		angles[1] = -Math.asin(A.r31); // Pitch

		// Gymbal lock: pitch = -90
		if (A.r31 == 1) {
			angles[0] = 0.0; // yaw = 0
			angles[2] = Math.atan2(-A.r12, -A.r13); // Roll
			System.out.println("Gimbal lock: pitch = -90");
		}

		// Gymbal lock: pitch = 90
		else if (A.r31 == -1) {
			angles[0] = 0.0; // yaw = 0
			angles[2] = Math.atan2(A.r12, A.r13); // Roll
			System.out.println("Gimbal lock: pitch = 90");
		}
		// General solution
		else {
			angles[0] = Math.atan2(A.r21, A.r11);
			angles[2] = Math.atan2(A.r32, A.r33);
			System.out.println("No gimbal lock");
		}
		return angles; // Euler angles in order yaw, pitch, roll
	}

	// code 4

	public static double[] rotatePoint(RotationMatrix A, double[] p_in) // Input coords in order (x,y,z)
	{
		double[] p_out = new double[3];
		p_out[0] = A.r11 * p_in[0] + A.r12 * p_in[1] + A.r13 * p_in[2];
		p_out[1] = A.r21 * p_in[0] + A.r22 * p_in[1] + A.r23 * p_in[2];
		p_out[2] = A.r31 * p_in[0] + A.r32 * p_in[1] + A.r33 * p_in[2];
		return p_out; // Output coords in order (x,y,z)
	}

	// code 5
	public RotationMatrix transposeMatrix(RotationMatrix A) {
		double[] firstMatrix = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		RotationMatrix B = new RotationMatrix(firstMatrix);
		B.r11 = A.r11;
		B.r12 = A.r21;
		B.r13 = A.r31;
		B.r21 = A.r12;
		B.r22 = A.r22;
		B.r23 = A.r32;
		B.r31 = A.r13;
		B.r32 = A.r23;
		B.r33 = A.r33;
		return B;
	}

	// code 6
	public static RotationMatrix multiplyMatrices(RotationMatrix A, RotationMatrix B) // Second rotation
	{
		double[] firstMatrix = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		RotationMatrix C = new RotationMatrix(firstMatrix);

		C.r11 = A.r11 * B.r11 + A.r12 * B.r21 + A.r13 * B.r31;
		C.r12 = A.r11 * B.r12 + A.r12 * B.r22 + A.r13 * B.r32;
		C.r13 = A.r11 * B.r13 + A.r12 * B.r23 + A.r13 * B.r33;
		C.r21 = A.r21 * B.r11 + A.r22 * B.r21 + A.r23 * B.r31;
		C.r22 = A.r21 * B.r12 + A.r22 * B.r22 + A.r23 * B.r32;
		C.r23 = A.r21 * B.r13 + A.r22 * B.r23 + A.r23 * B.r33;
		C.r31 = A.r31 * B.r11 + A.r32 * B.r21 + A.r33 * B.r31;
		C.r32 = A.r31 * B.r12 + A.r32 * B.r22 + A.r33 * B.r32;
		C.r33 = A.r31 * B.r13 + A.r32 * B.r23 + A.r33 * B.r33;
		return C; // Returns combined rotation (C=AB)
	}


	
	public static String displayVector(double[] v) {
		final String sp = " ";
		final StringBuilder s = new StringBuilder();
		s.append("[").append(v[0]).append(sp).append(v[1]).append(sp).append(v[2]).append("]");

		return s.toString();
	}
	
	public static void main(String[] args) {
		double[] v = { 1, 1, 1 }; // vector v to rotate
		double[] resultv = new double[3];
		double[] elements = { 1, 0, 0, 0, 0, -1, 0, 1, 0 };
		//RotationMatrix rm = new RotationMatrix(elements); //the rotation matrix to rotate 90 degrees over x asix		
		double[] elements1 = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
		RotationMatrix rm1 = new RotationMatrix(elements1);
				
		//get memory used
		Runtime rt = Runtime.getRuntime();
	    long prevFree = rt.freeMemory();
		//get run time
		long start = System.currentTimeMillis();
		final int CAP = 100_000; //100_000_000
		for (int i = 0; i < CAP; i++)
		{
			RotationMatrix rmx = new RotationMatrix(elements);		//the rotation matrix over x asix
			RotationMatrix rmy = new RotationMatrix(elements1);		//the rotation matrix over y asix
			RotationMatrix rmz = new RotationMatrix(elements1);		//the rotation matrix over z asix
			RotationMatrix rm = multiplyMatrices(multiplyMatrices(rmx,rmy),rmz);		//the resulting rotation matrix over x,y,z matrices 
			resultv = rotatePoint(rm, v);			//rotate v over x,y,z axis
			v = resultv;
		}
		System.out.println("the result vector is " + displayVector(resultv));
		long end = System.currentTimeMillis();
		long diff = end - start;
		System.out.println("rotation using matrices takes: "+ diff + " milliseconds");
		
		long free = rt.freeMemory();
		long memoryused = (prevFree - free)/1000000;
        System.out.println("rotation using matrices takes: "+ memoryused + " Megabytes");
	}
}
