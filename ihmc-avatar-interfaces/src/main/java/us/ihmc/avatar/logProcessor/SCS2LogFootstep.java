package us.ihmc.avatar.logProcessor;

import us.ihmc.robotics.robotSide.RobotSide;

public class SCS2LogFootstep
{
   private double time;
   private RobotSide side;
   private double[] polygon;

   public SCS2LogFootstep(double time, RobotSide side, double[] polygon)
   {
      this.side = side;
      this.polygon = polygon.clone();
   }

   public RobotSide getSide()
   {
      return side;
   }

   public double[] getPolygon()
   {
      return polygon;
   }
}
