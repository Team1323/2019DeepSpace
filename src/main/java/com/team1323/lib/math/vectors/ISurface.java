package com.team1323.lib.math.vectors;

import java.util.function.Function;

import com.team254.lib.geometry.Translation2d;

public interface ISurface {
	public abstract Function<Translation2d,Double> f();
	public abstract Function<Translation2d,Double> dfdx();
	public abstract Function<Translation2d,Double> dfdy();
}