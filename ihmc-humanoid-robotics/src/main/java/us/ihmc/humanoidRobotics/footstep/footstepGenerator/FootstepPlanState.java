package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
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
      footstepData.getLocation().set(new Point3D(x, y, 0));
      
      Quaternion orientation = new Quaternion();
      orientation.setToYawOrientation(theta);
      footstepData.getOrientation().set(orientation);
      footstepData.setRobotSide(side.toByte());
      this.theta = theta;
   }

   public FootstepPlanState(FootstepPlanState copy)
   {
      this(new FootstepDataMessage(copy.footstepData), copy.theta);
      this.terrainType = copy.terrainType;
   }

   public void setHeight(double z) { this.footstepData.getLocation().setZ(z);}

   public Point3D getPosition() { return footstepData.getLocation();}

   public boolean equals(Object o)
   {
      FootstepPlanState other = (FootstepPlanState) o;
      boolean goodMatch = ((discretize(footstepData.getLocation().getX(), distanceResolution) == discretize(other.footstepData.getLocation().getX(), distanceResolution))
            && (discretize(footstepData.getLocation().getY(), distanceResolution) == discretize(other.footstepData.getLocation().getY(), distanceResolution))
            && (discretize(theta, thetaResolution) == discretize(other.theta, thetaResolution))
            && (footstepData.getRobotSide() == other.footstepData.getRobotSide()));
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
      return footstepData.getLocation().distance(other.footstepData.getLocation());
   }

   public double horizonalDistance(FootstepPlanState other)
   {
      return Math.sqrt(Math.pow(footstepData.getLocation().getX() - other.footstepData.getLocation().getX(), 2) + Math.pow(footstepData.getLocation().getY() - other.footstepData.getLocation().getY(), 2));
   }


   public Point3D getPoint3d()
   {
      return footstepData.getLocation();
   }

   public RobotSide getRobotSide()
   {
      return RobotSide.fromByte(footstepData.getRobotSide());
   }

   @Override
   public int compareTo(FootstepPlanState s)
   {
      if (this.equals(s)) return 0;
      if (this.footstepData.getLocation().getX() < s.footstepData.getLocation().getX())
         return -1;
      if (this.footstepData.getLocation().getX() > s.footstepData.getLocation().getX())
         return 1;
      if (this.footstepData.getLocation().getY() < s.footstepData.getLocation().getY())
         return -1;
      if (this.footstepData.getLocation().getY() > s.footstepData.getLocation().getY())
         return 1;
      if (this.footstepData.getLocation().getZ() < s.footstepData.getLocation().getZ())
         return -1;
      if (this.footstepData.getLocation().getZ() > s.footstepData.getLocation().getZ())
         return 1;
      if (this.theta < s.theta)
         return -1;
      if (this.theta > s.theta)
         return 1;
      if (this.footstepData.getRobotSide() < s.footstepData.getRobotSide())
         return -1;
      if (this.footstepData.getRobotSide() > s.footstepData.getRobotSide())
         return 1;
      return 0;
   }

   @Override
   public int hashCode()
   {
      return (int) (discretize(footstepData.getLocation().getX(), distanceResolution) + 3424.0 * discretize(footstepData.getLocation().getY(), distanceResolution) + 16353.0 * discretize(theta, thetaResolution) + 72432.0 * RobotSide.fromByte(footstepData.getRobotSide()).ordinal());
   }

   private double discretize(double value, double interval)
   {
      return interval * (int) (value / interval + 0.5);
   }

   public String getID()
   {
      String id = "";
      id = RobotSide.fromByte(footstepData.getRobotSide()).getSideNameFirstLetter() + "_" + discretize(footstepData.getLocation().getX(), distanceResolution) + "_" + discretize(footstepData.getLocation().getY(), distanceResolution) + "_" + discretize(theta, thetaResolution);
      return id;
   }
}
