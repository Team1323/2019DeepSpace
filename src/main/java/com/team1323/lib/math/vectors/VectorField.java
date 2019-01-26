package com.team1323.lib.math.vectors;

import com.team254.lib.geometry.Translation2d;

public abstract class VectorField implements IVectorField {
	public abstract Translation2d getVector(Translation2d here);
	public VectorField add(VectorField other) {
		VectorField temp = this;
		return new VectorField() {
			@Override
			public Translation2d getVector(Translation2d here) {
			//	System.out.println("First: "+temp.getVector(here).x()+" "+temp.getVector(here).y());
			//	System.out.println("Secnd: "+other.getVector(here).x()+" "+other.getVector(here).y());
				return temp.getVector(here).translateBy(other.getVector(here)).normalize();
			}
		};
	}
	public VectorField inverse() {
		VectorField temp = this;
		return new VectorField() {
			@Override
			public Translation2d getVector(Translation2d here) {
				return temp.getVector(here).inverse();
			}
		};
	}
	public VectorField scale(double s) {
		VectorField temp = this;
		return new VectorField() {
			@Override
			public Translation2d getVector(Translation2d here) {
				return temp.getVector(here).scale(s);
			}
		};
	}
	// the following is not real curvature, just a cheap attempt at guessing it
	public double getCurvatureAhead(Translation2d here, double step) {
		Translation2d out_here = getVector(here);
		Translation2d there = here.translateBy(out_here.scale(step));
		Translation2d out_there = getVector(there);
		return Translation2d.getAngle(out_here,out_there).getRadians();
	}
}