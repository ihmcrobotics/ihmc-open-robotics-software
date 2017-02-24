package us.ihmc.avatar.manipulating;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.robotics.geometry.transformables.Pose;



public interface ConstrainedWayPointGenerator
{      
   public void setInitialPose(Pose pose);
   public void setFinalPose(Pose pose);
   
   public void setNumberOfWayPoints(int numPoints);   

   public void initializeWayPoints();
   public void generateWayPoints();
   public Point3d getWayPointPosition(int indexOfWayPoint);
   public Quat4d getWayPointOrientation(int indexOfWayPoint);
   public Pose getWayPoint(int indexOfWayPoint);
   public ArrayList<Graphics3DObject> createXYZAxisOfWayPoint(int indexOfWayPoint);
   
   // true : valid waypoint, false : invalid waypoint
   public boolean getResultForPoint(Pose pose);
   public boolean getResultForPoint(Point3d wayPointPosition, Quat4d wayPointOrientation);
   public boolean getResultForPoint(int indexOfWayPoint);
   
   public void setWayPoint(int indexOfWayPoint, Pose pose);
   public void reGenerateWayPoints(int startIndexOfWayPoint);
   
   
}
