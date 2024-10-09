package us.ihmc.avatar.logProcessor;

import us.ihmc.robotics.robotSide.RobotSide;

public class SCS2LogFootstep
{
   private final double time;
   private final RobotSide side;
   private final double[] polygon;

   public SCS2LogFootstep(double time, RobotSide side, double[] polygon)
   {
      this.time = time;
      this.side = side;
      this.polygon = polygon.clone();
   }

   public double getTime()
   {
      return time;
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
