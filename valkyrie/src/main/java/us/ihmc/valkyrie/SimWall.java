package us.ihmc.valkyrie;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

public class SimWall {
	public String name;
	public double[] center;
	public double[] lengths;
	public double slopeRadians;
	public double yawDegrees;
	public AppearanceDefinition appearance;
	
	public SimWall() {
		name = "wall";
		center = new double[]{ 0, 0, 0 };
		lengths = new double[] {0, 0, 0 };
		slopeRadians = 0;
		yawDegrees = 0;
		appearance = YoAppearance.ForestGreen();
	}
	
	public SimWall(double xCenter, double yCenter, double zCenter, 
			       double xLength, double yLength, double zLength) {
		this(xCenter, yCenter, zCenter, xLength, yLength, zLength, 0, 0);
	}
	
	public SimWall(double xCenter, double yCenter, double zCenter, 
			       double xLength, double yLength, double zLength, 
			       double slopeRadians, double yawDegrees) {
		this(xCenter, yCenter, zCenter, xLength, yLength, zLength, slopeRadians, yawDegrees, YoAppearance.Grey());
	}
	
	public SimWall(double xCenter, double yCenter, double zCenter, 
		       double xLength, double yLength, double zLength, 
		       double slopeRadians, double yawDegrees, AppearanceDefinition appearance) {
		this.center = new double[] {xCenter, yCenter, zCenter};
		this.lengths = new double[] {xLength, yLength, zLength};
		this.slopeRadians = slopeRadians;
		this.yawDegrees = yawDegrees;
		this.appearance = appearance;
	}
	
	public void SetName(String name) {
		this.name = name;
	}
}
