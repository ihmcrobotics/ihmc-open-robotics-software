package us.ihmc.robotics.dataStructures;

import us.ihmc.robotics.geometry.InclusionFunction;

import javax.vecmath.Point3d;
import java.util.List;

public interface HeightMapWithPoints
{
   abstract double getHeightAtPoint(double x, double y);
   abstract boolean addPoint(double x, double y, double z);
   abstract boolean containsPoint(double x, double y);
   abstract void clear();
   abstract List<Point3d> getAllPointsWithinArea(double xCenter, double yCenter, double xExtent, double yExtent);
   abstract List<Point3d> getAllPointsWithinArea(double xCenter, double yCenter, double xExtent, double yExtent,
         InclusionFunction<Point3d> maskFunctionAboutCenter);
}
