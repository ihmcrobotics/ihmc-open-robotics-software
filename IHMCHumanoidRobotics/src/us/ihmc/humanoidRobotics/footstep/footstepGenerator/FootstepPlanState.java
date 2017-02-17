package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
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
   public FootstepDataMessage footstepData = new FootstepDataMessage();
   public double theta;
   public TerrainType terrainType = TerrainType.UNKNOWN;
   public double supportPolygonArea = 0;


   public FootstepPlanState(FootstepDataMessage startState, double theta)
   {
      footstepData = startState;
      this.theta = theta;
   }

   public FootstepPlanState(double x, double y, double theta, RobotSide side)
   {
      footstepData.location = new Point3D(x, y, 0);
      footstepData.orientation = new Quaternion();
      footstepData.orientation.setToYawQuaternion(theta);
      footstepData.robotSide = side;
      this.theta = theta;
   }

   public FootstepPlanState(FootstepPlanState copy)
   {
      this(copy.footstepData.clone(), copy.theta);
      this.terrainType = copy.terrainType;
   }

   public void setHeight(double z) { this.footstepData.location.setZ(z);}

   public Point3D getPosition() { return footstepData.location;}

   public boolean equals(Object o)
   {
      FootstepPlanState other = (FootstepPlanState) o;
      boolean goodMatch = ((discretize(footstepData.location.getX(), distanceResolution) == discretize(other.footstepData.location.getX(), distanceResolution))
            && (discretize(footstepData.location.getY(), distanceResolution) == discretize(other.footstepData.location.getY(), distanceResolution))
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
      return Math.sqrt(Math.pow(footstepData.location.getX() - other.footstepData.location.getX(), 2) + Math.pow(footstepData.location.getY() - other.footstepData.location.getY(), 2));
   }


   public Point3D getPoint3d()
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
      if (this.footstepData.location.getX() < s.footstepData.location.getX())
         return -1;
      if (this.footstepData.location.getX() > s.footstepData.location.getX())
         return 1;
      if (this.footstepData.location.getY() < s.footstepData.location.getY())
         return -1;
      if (this.footstepData.location.getY() > s.footstepData.location.getY())
         return 1;
      if (this.footstepData.location.getZ() < s.footstepData.location.getZ())
         return -1;
      if (this.footstepData.location.getZ() > s.footstepData.location.getZ())
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
      return (int) (discretize(footstepData.location.getX(), distanceResolution) + 3424.0 * discretize(footstepData.location.getY(), distanceResolution) + 16353.0 * discretize(theta, thetaResolution) + 72432.0 * footstepData.robotSide.ordinal());
   }

   private double discretize(double value, double interval)
   {
      return interval * (int) (value / interval + 0.5);
   }

   public String getID()
   {
      String id = "";
      id = footstepData.robotSide.getSideNameFirstLetter() + "_" + discretize(footstepData.location.getX(), distanceResolution) + "_" + discretize(footstepData.location.getY(), distanceResolution) + "_" + discretize(theta, thetaResolution);
      return id;
   }
}
