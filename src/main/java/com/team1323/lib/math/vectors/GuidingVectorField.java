package com.team1323.lib.math.vectors;

import java.util.function.Function;

import com.team254.lib.geometry.Translation2d;

public class GuidingVectorField extends VectorField {
	// Implement as e.g.
	// GuidingVectorField(Translation2d here -> here.x()+here.y(), ... )
/*	public GuidingVectorField(Function<Translation2d,Double> surface,
				  Function<Translation2d,Double> dfdx,
				  Function<Translation2d,Double> dfdy) {
		phi = surface;
		dfdx_ = dfdx;
		dfdy_ = dfdy;
	}
	public GuidingVectorField(Function<Translation2d,Double> surface,
				  Function<Translation2d,Double> dfdx,
				  Function<Translation2d,Double> dfdy,
				  boolean isReversed) {
		phi = surface;
		dfdx_ = dfdx;
		dfdy_ = dfdy;
		if(isReversed) direction = -1;
	}*/
	public GuidingVectorField(Surface surface_, boolean isReversed, Function<Translation2d,Double> k_) {
		surface = surface_;
		if(isReversed) direction = -1;
		k = k_;
	}
	public GuidingVectorField(Surface surface_) {
		this(surface_, false, here -> 1.0);
	}
	public GuidingVectorField(Surface surface_, boolean isReversed) {
		this(surface_, isReversed, here -> 1.0);
	}
	public GuidingVectorField(Surface surface_, Function<Translation2d,Double> k_) {
		this(surface_, false, k_);
	}
	
		protected int direction = 1;
/*	protected Function<Translation2d,Double> phi;
	protected Function<Translation2d,Double> dfdx_;
	protected Function<Translation2d,Double> dfdy_;*/
	protected Surface surface;
	protected Translation2d n(Translation2d here) {
		Translation2d nv = new Translation2d(surface.dfdx().apply(here),surface.dfdy().apply(here));
//		System.out.println("n" + here + " = " + nv);
		return nv;
	}
	// Is psi even necessary? Since k is a function now, idk
	protected double psi(double t) { return t; } // FIXME: pass this in constructor, definitely (type is known)
	protected double e(Translation2d here) { return psi(surface.f().apply(here)); } // surface.f is phi from the paper
	protected Function<Translation2d,Double> k;
	protected Translation2d tau(Translation2d here) {
		Translation2d nv = n(here);
		Translation2d tv = new Translation2d(nv.y()*direction,-nv.x()*direction);
//		System.out.println("t" + here + " = " + tv);
		return tv;
	}
	public Translation2d getVector(Translation2d here) {
		return (new Translation2d(n(here).scale(-k.apply(here)*e(here)),tau(here))).normalize();
	}
}