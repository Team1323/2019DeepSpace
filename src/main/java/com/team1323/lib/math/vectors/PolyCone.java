package com.team1323.lib.math.vectors;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// ok bois
/* GuidingVectorField switchAvoidanceCW = new GuidingVectorField(new PolyCone(new ArrayList<Translation2d> {
		Constants.kLeftSwitchCloseCorner,
		Constants.kLeftSwitchFarCorner,
		Constants.kRightSwitchFarCorner,
		Constants.kRightSwitchCloseCorner
	}));
*/
public class PolyCone extends Surface {
	// first point lies on z=-1
	// second and third points lie on z=0
	// therefore z1-z0 and z2-z0 are both 1
	public class PlaneSegment extends Surface {
		public PlaneSegment(Translation2d p0, Translation2d p1, Translation2d p2) {
			this(p0,p1,p2,Double.POSITIVE_INFINITY);
		}
		public PlaneSegment(Translation2d p0, Translation2d p1, Translation2d p2, double radius) {
	
			SmartDashboard.putNumber("Path X", p0.x());
			SmartDashboard.putNumber("Path Y", p0.y());
			Timer.delay(0.1);
			SmartDashboard.putNumber("Path X", p1.x());
			SmartDashboard.putNumber("Path Y", p1.y());
			Timer.delay(0.1);
			SmartDashboard.putNumber("Path X", p2.x());
			SmartDashboard.putNumber("Path Y", p2.y());
			Timer.delay(0.1);
			SmartDashboard.putNumber("Path X", p0.x());
			SmartDashboard.putNumber("Path Y", p0.y());
			Timer.delay(0.1);

			
			Translation2d v1 = new Translation2d(p0,p1);
			Translation2d v2 = new Translation2d(p0,p2);
			double a = v1.y()-v2.y();
			double b = v2.x()-v1.x();
			double c = Translation2d.cross(v1,v2);
			
			dfdx_ = (here -> a);
			dfdy_ = (here -> b);
			
			f_ = (here -> {
				// the following return is the value of the plane z(x,y)
				if(here.isWithinAngle(p1,p0,p2) && here.distanceToLine(p1,p2) <= radius) return (a*(p0.x()-here.x())+b*(p0.y()-here.y()))/c - 1.0;
				else return 0.0;
			});
			dfdx_ = (here -> {
				if(here.isWithinAngle(p1,p0,p2) && here.distanceToLine(p1,p2) <= radius) return a;
				else return 0.0;
			});
			dfdy_ = (here -> {
				if(here.isWithinAngle(p1,p0,p2) && here.distanceToLine(p1,p2) <= radius) return b;
				else return 0.0;
			});
		}
		public Function<Translation2d,Double> f_;
		public Function<Translation2d,Double> dfdx_;
		public Function<Translation2d,Double> dfdy_;
		public Function<Translation2d,Double> f() {return f_;}
		public Function<Translation2d,Double> dfdx() {return dfdx_;}
		public Function<Translation2d,Double> dfdy() {return dfdy_;}
		
	} // end of plane ting
	// I don't want to have to throw anything, so please construct responsibly!
	// Don't try to create a polygon with fewer than three sides.
	public PolyCone(List<Translation2d> vertices) {
		this(Double.POSITIVE_INFINITY, vertices);
	}
	public PolyCone(double radius, List<Translation2d> vertices) {
			Translation2d center = new Translation2d(0.0,0.0);
		// first find center point
		for(int i = 0; i < vertices.size(); i++) {
			center = center.translateBy(vertices.get(i));
		}
		center = center.scale(1.0/(double)vertices.size());
		System.out.println("Center: "+center);
		// now generate list of sides
		for(int i = 0; i < vertices.size()-1; i++) {
			sides.add(new PlaneSegment(center, vertices.get(i), vertices.get(i+1), radius));
		}
		sides.add(new PlaneSegment(center, vertices.get(vertices.size()-1), vertices.get(0), radius));
	}
	protected ArrayList<PlaneSegment> sides = new ArrayList<>();
	public double f(Translation2d here) {
//		System.out.print("f"+here+" = 0.0");
		double z = 0.0;
		for(int i = 0; i < sides.size(); i++) {
			double k = sides.get(i).f().apply(here);
//			System.out.print(" + " + (new DecimalFormat("#0.000")).format(k));
			z += k;
		}
//		System.out.println("");
		return z;
	}
	public double dfdx(Translation2d here) {
		System.out.print("dfdx"+here+" = 0.0");
		double z = 0.0;
		for(int i = 0; i < sides.size(); i++) {
			double k = sides.get(i).dfdx().apply(here);
			System.out.print(" + " + (new DecimalFormat("#0.000")).format(k));
			z += k;
		}
		System.out.println("");
		return z;
	}
	public double dfdy(Translation2d here) {
		System.out.print("dfdy"+here+" = 0.0");
		double z = 0.0;
		for(int i = 0; i < sides.size(); i++) {
			double k = sides.get(i).dfdy().apply(here);
			System.out.print(" + " + (new DecimalFormat("#0.000")).format(k));
			z += k;
		}
		System.out.println("");
		return z;
	}
	public Function<Translation2d,Double> f_;
	public Function<Translation2d,Double> dfdx_;
	public Function<Translation2d,Double> dfdy_;
	public Function<Translation2d,Double> f() {return (here -> f(here));}
	public Function<Translation2d,Double> dfdx() {return (here -> dfdx(here));}
	public Function<Translation2d,Double> dfdy() {return (here -> dfdy(here));}

}