package us.ihmc.humanoidRobotics.footstep.footsepGenerator;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepData;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepPlanState implements Comparable<FootstepPlanState>
{
   public enum TerrainType
   {
      UNKNOWN, PLANAR, CONVEX_HULL, UNKNOWN_IN_GRID, IMPASSIBLE, OBSTACLE
   }

   private static final long serialVersionUID = 1L;

   //discretization information
   static double epsilon = 0.0003;
   static double distanceResolution = 0.03;
   static double thetaResolution = Math.PI / 16;

   //position information
   public FootstepData footstepData = new FootstepData();
   public double theta;
   public TerrainType terrainType = TerrainType.UNKNOWN;
   public double supportPolygonArea = 0;


   public FootstepPlanState(FootstepData startState, double theta)
   {
      footstepData = startState;
      this.theta = theta;
   }

   public FootstepPlanState(double x, double y, double theta, RobotSide side)
   {
      footstepData.location = new Point3d(x, y, 0);
      footstepData.orientation = new Quat4d();
      RotationFunctions.setQuaternionBasedOnYawPitchRoll(footstepData.orientation, theta, 0, 0);
      footstepData.robotSide = side;
      this.theta = theta;
   }

   public FootstepPlanState(FootstepPlanState copy)
   {
      this(copy.footstepData.clone(), copy.theta);
      this.terrainType = copy.terrainType;
   }

   public void setHeight(double z) { this.footstepData.location.z = z;}

   public Point3d getPosition() { return footstepData.location;}

   public boolean equals(Object o)
   {
      FootstepPlanState other = (FootstepPlanState) o;
      boolean goodMatch = ((discretize(footstepData.location.x, distanceResolution) == discretize(other.footstepData.location.x, distanceResolution))
            && (discretize(footstepData.location.y, distanceResolution) == discretize(other.footstepData.location.y, distanceResolution))
            && (discretize(theta, thetaResolution) == discretize(other.theta, thetaResolution))
            && (footstepData.robotSide == other.footstepData.robotSide));
      if (goodMatch)
      {
         return true;
      }
      return false;
   }

   public FootstepPlanState clone()
   {
      return new FootstepPlanState(this);
   }

   @Override
   public String toString()
   {
      return footstepData.toString() + terrainType;
   }

   public double distance(FootstepPlanState other)
   {
      return footstepData.location.distance(other.footstepData.location);
   }

   public double horizonalDistance(FootstepPlanState other)
   {
      return Math.sqrt(Math.pow(footstepData.location.x - other.footstepData.location.x, 2) + Math.pow(footstepData.location.y - other.footstepData.location.y, 2));
   }


   public Point3d getPoint3d()
   {
      return footstepData.location;
   }

   public RobotSide getRobotSide()
   {
      return footstepData.robotSide;
   }

   @Override
   public int compareTo(FootstepPlanState s)
   {
      if (this.equals(s)) return 0;
      if (this.footstepData.location.x < s.footstepData.location.x)
         return -1;
      if (this.footstepData.location.x > s.footstepData.location.x)
         return 1;
      if (this.footstepData.location.y < s.footstepData.location.y)
         return -1;
      if (this.footstepData.location.y > s.footstepData.location.y)
         return 1;
      if (this.footstepData.location.z < s.footstepData.location.z)
         return -1;
      if (this.footstepData.location.z > s.footstepData.location.z)
         return 1;
      if (this.theta < s.theta)
         return -1;
      if (this.theta > s.theta)
         return 1;
      if (this.footstepData.robotSide.ordinal() < s.footstepData.robotSide.ordinal())
         return -1;
      if (this.footstepData.robotSide.ordinal() > s.footstepData.robotSide.ordinal())
         return 1;
      return 0;
   }

   @Override
   public int hashCode()
   {
      return (int) (discretize(footstepData.location.x, distanceResolution) + 3424.0 * discretize(footstepData.location.y, distanceResolution) + 16353.0 * discretize(theta, thetaResolution) + 72432.0 * footstepData.robotSide.ordinal());
   }

   private double discretize(double value, double interval)
   {
      return interval * (int) (value / interval + 0.5);
   }

   public String getID()
   {
      String id = "";
      id = footstepData.robotSide.getSideNameFirstLetter() + "_" + discretize(footstepData.location.x, distanceResolution) + "_" + discretize(footstepData.location.y, distanceResolution) + "_" + discretize(theta, thetaResolution);
      return id;
   }
}
