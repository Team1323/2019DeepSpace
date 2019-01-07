package com.team1323.lib.math.vectors;

import java.util.function.Function;
import com.team254.lib.geometry.Translation2d;

public class GuidingPolynomialVectorField extends VectorField {

	// Create a directed line that contains the point (x,y) and has a given heading
/*	public GuidingPolynomialVectorField(Translation2d where, Rotation2d heading) {
		fc = new double[]{where.y() - heading.tan()*where.x(),heading.tan()};
		setDfc(fc);
		if(heading.getRadians() >= Math.PI) direction = -1;
	}*/
	public GuidingPolynomialVectorField(double[] coeffs, boolean isReversed, Function<Translation2d,Double> k_) {
		fc = coeffs;
		setDfc(fc);
		if(isReversed) direction = -1;
		k = k_;
	}
	public GuidingPolynomialVectorField(double[] coeffs) {
		this(coeffs, false, here -> 1.0);
	}
	public GuidingPolynomialVectorField(double[] coeffs, Function<Translation2d,Double> k_) {
		this(coeffs, false, k_);
	}
	protected void setDfc(double[] fc) {
		dfc = new double[fc.length - 1];
		for(int i = 0; i < dfc.length; i++) {
			dfc[i] = fc[i+1]*(i+1);
		}
	}
	protected int direction = 1;
	protected double[] fc; // f[n] is coefficient for x^n term
	protected double[] dfc;
	protected double f(double[] cs, double x) {
		int deg = cs.length-1;
		double val = cs[deg];
		while(deg > 0) {
			val *= x;
			val += cs[--deg];
		}
		return val;
	}

	protected double phi(Translation2d here) { return here.y() - f(fc,here.x()); }
	protected Translation2d n(Translation2d here) { return new Translation2d(-f(dfc,here.x()),1); }
	protected double psi(double t) { return t; }
	protected double e(Translation2d here) { return psi(phi(here)); }
	/** Lower values of k mean more following, less approaching */
	protected Function<Translation2d,Double> k;
	protected Translation2d tau(Translation2d here) {
		Translation2d nv = n(here);
		return new Translation2d(nv.y()*direction,-nv.x()*direction);
	}
	public Translation2d getVector(Translation2d here) {
		return (new Translation2d(n(here).scale(k.apply(here)*e(here)),tau(here))).normalize();
	}
}