package us.ihmc.robotics.dataStructures;

import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.InclusionFunction;

public interface HeightMapWithPoints
{
   abstract double getHeightAtPoint(double x, double y);
   abstract boolean addPoint(double x, double y, double z);
   abstract boolean containsPoint(double x, double y);
   abstract void clear();
   abstract List<Point3D> getAllPointsWithinArea(double xCenter, double yCenter, double xExtent, double yExtent);
   abstract List<Point3D> getAllPointsWithinArea(double xCenter, double yCenter, double xExtent, double yExtent,
         InclusionFunction<Point3D> maskFunctionAboutCenter);
}
