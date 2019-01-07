package com.team1323.lib.math.vectors;

import java.util.List;

import com.team254.lib.geometry.Translation2d;

/**
 * This class represents a vector field that moves around a given polygon.
 * Along the sides, the field is parallel to the side.
 * Around the vertices, the field is rotational.
 * Intended use is for obstacle avoidance.
 * 
 * @author Joseph Reed
 */
public class PolyCirculationVectorField extends VectorField {
	
	public PolyCirculationVectorField(List<Translation2d> vertices_) {
		this(vertices_,Double.POSITIVE_INFINITY);
	}
	public PolyCirculationVectorField(List<Translation2d> vertices_, double radius_) {
		radius = radius_;
		center = new Translation2d(0.0,0.0);
		for(Translation2d v : vertices_) {
			vertices.add(v);
			center = center.translateBy(v);
		}
		center = center.scale(1.0/(double)vertices.size());
		vertices.add(vertices_.get(0)); // add first last so we won't have to connect them manually
		vertices.add(vertices_.get(1)); // add second last so we can uhhh like use three points in our corner checking code
	}
	protected double radius;
	protected Translation2d center;
	protected List<Translation2d> vertices;
	public Translation2d getVector(Translation2d here) {
		// General approach:
		// 1. Check whether here is in a corner region. If so, return <-y,x>.
		// 2. Find the region containing here. Return a vector parallel to its defining side.
		
		// Check whether here is in a corner region
		// i = 1 to check (0,1,2); 
		// i = sz - 2 to check (sz-3, sz-2, sz-1), i.e., (-1,0,1)
		for(int i = 1; i < vertices.size() - 1; i++) {
			// Translate our angle to have its center at the origin
			Translation2d here_ = here.translateBy(vertices.get(i).inverse());
			if(here.isWithinAngle(
					vertices.get(i-1),
					vertices.get(i),
					vertices.get(i+1),
					true // check within vertical angle
				)) {
				if(here_.norm() <= radius) {
					return (new Translation2d(-here_.y(),here_.x())).normalize(); // use translated position because it will rotate about the vertex
				} else {
					return Translation2d.identity();
				}
			}
		}
		// If we get here, we ain't in a corner region
		// Find the region containing here
		for(int i = 0; i < vertices.size() - 1; i++) {
			if(here.isWithinAngle(
					vertices.get(i),
					center,
					vertices.get(i+1)
				)) {
				if(here.distanceToLine(vertices.get(i),vertices.get(i+1)) <= radius) {
					return (new Translation2d(vertices.get(i),vertices.get(i+1))).normalize();
				} else {
					return Translation2d.identity();
				}
			}
		}
		// If we get here, something is horribly wrong
		return Translation2d.identity();
	}
}