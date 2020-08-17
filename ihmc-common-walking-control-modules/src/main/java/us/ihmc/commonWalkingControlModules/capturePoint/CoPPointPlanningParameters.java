package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector2DBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.variable.YoDouble;

public class CoPPointPlanningParameters
{
   private final SideDependentList<YoFrameVector2D> copOffsets = new SideDependentList<>();

   private double stepLengthToCoPOffsetFactor;

   private YoDouble maxCoPOffset;
   private YoDouble minCoPOffset;


   public void setStepLengthToCoPOffsetFactor(double stepLengthToCoPOffsetFactor)
   {
      this.stepLengthToCoPOffsetFactor = stepLengthToCoPOffsetFactor;
   }

   public void setCoPOffsets(RobotSide robotSide, YoFrameVector2D copOffsets)
   {
      this.copOffsets.put(robotSide, copOffsets);
   }

   public void setCoPOffsetBounds(YoDouble minCoPOffset, YoDouble maxCoPOffset)
   {
      this.minCoPOffset = minCoPOffset;
      this.maxCoPOffset = maxCoPOffset;
   }

   public double getStepLengthToCoPOffsetFactor()
   {
      return stepLengthToCoPOffsetFactor;
   }

   public FixedFrameVector2DBasics getCoPOffsets(RobotSide robotSide)
   {
      return copOffsets.get(robotSide);
   }

   public YoDouble getMaxCoPOffset()
   {
      return maxCoPOffset;
   }

   public YoDouble getMinCoPOffset()
   {
      return minCoPOffset;
   }
}
