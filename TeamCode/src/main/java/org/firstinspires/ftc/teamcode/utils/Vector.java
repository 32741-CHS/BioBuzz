package org.firstinspires.ftc.teamcode.utils;

import java.lang.Math;

public class Vector {
	public static Vector3 create(double x, double y, double z) {
		return new Vector3(x, y, z);
	}

	public static Vector2 create(double x, double y) {
		return new Vector2(x, y);
	}

	public static Vector3 Add(Vector3 v1, Vector3 v2) {
		return new Vector3(
			v1.x + v2.x,
			v1.y + v2.y,
			v1.z + v2.z
		);
	}

	public static Vector2 Add(Vector2 v1, Vector2 v2) {
		return new Vector2(
			v1.x + v2.x,
			v1.y + v2.y
		);
	}

	public static Vector3 Sub(Vector3 v1, Vector3 v2) {
		return new Vector3(
			v1.x - v2.x,
			v1.y - v2.y,
			v1.z - v2.z
		);
	}


	public static Vector2 Sub(Vector2 v1, Vector2 v2) {
		return new Vector2(
			v1.x - v2.x,
			v1.y - v2.y
		);
	}

	public static double Magnitude(Vector2 v) {
		return Math.sqrt(v.x) + Math.sqrt(v.y);
	}

	public static double Magnitude(Vector3 v) {
		return Math.sqrt(v.x) + Math.sqrt(v.y) + Math.sqrt(v.z);
	}

	public static Vector3 Normalize(Vector3 v) {
		double mag = Vector.Magnitude(v);
		return new Vector3(
			v.x / mag,
			v.y / mag,
			v.z / mag
		);
	};

	public static Vector2 Normalize(Vector2 v) {
		double mag = Vector.Magnitude(v);
		return new Vector2(
			v.x / mag,
			v.y / mag
		);
	};

};	

class Vector3 {
	public double x;
	public double y;
	public double z;

	public Vector3(double x1, double y1, double z1) {
		x = x1;
		y = y1;
		z = z1;
	}

};

class Vector2 {
	public double x;
	public double y;

	public Vector2(double x1, double y1) {
		x = x1;
		y = y1;
	}
	
};

