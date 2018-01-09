package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.CoPSupportPolygonNames;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.variable.YoDouble;

public class CoPPointPlanningParameters
{
   private final CoPPointName copPointName;
   private final SideDependentList<YoFrameVector2d> copOffsets = new SideDependentList<>();

   private boolean isConstrainedToMinMax;
   private boolean isConstrainedToSupportPolygon;
   private CoPSupportPolygonNames stepLengthOffsetPolygon;
   private double stepLengthToCoPOffsetFactor;
   private Vector2D copOffsetsInFootFrame;
   private CoPSupportPolygonNames supportPolygonName;
   private Vector2D copOffsetBoundsInFootFrame;

   private YoDouble maxCoPOffset;
   private YoDouble minCoPOffset;

   public CoPPointPlanningParameters(CoPPointName copPointName)
   {
      this.copPointName = copPointName;
   }

   public void setIsConstrainedToMinMax(boolean isConstrainedToMinMax)
   {
      this.isConstrainedToMinMax = isConstrainedToMinMax;
   }

   public void setIsConstrainedToSupportPolygon(boolean isConstrainedToSupportPolygon)
   {
      this.isConstrainedToSupportPolygon = isConstrainedToSupportPolygon;
   }

   public void setStepLengthToCoPOffsetFactor(double stepLengthToCoPOffsetFactor)
   {
      this.stepLengthToCoPOffsetFactor = stepLengthToCoPOffsetFactor;
   }

   public void setStepLengthOffsetPolygon(CoPSupportPolygonNames stepLengthOffsetPolygon)
   {
      this.stepLengthOffsetPolygon = stepLengthOffsetPolygon;
   }

   public void setCopOffsetsInFootFrame(Vector2D copOffsetsInFootFrame)
   {
      this.copOffsetsInFootFrame = copOffsetsInFootFrame;
   }

   public void setSupportPolygonName(CoPSupportPolygonNames supportPolygonName)
   {
      this.supportPolygonName = supportPolygonName;
   }

   public void setCopOffsetBoundsInFootFrame(Vector2D copOffsetBoundsInFootFrame)
   {
      this.copOffsetBoundsInFootFrame = copOffsetBoundsInFootFrame;
   }

   public void setCoPOffsets(RobotSide robotSide, YoFrameVector2d copOffsets)
   {
      this.copOffsets.put(robotSide, copOffsets);
   }

   public void setCoPOffsetBounds(YoDouble minCoPOffset, YoDouble maxCoPOffset)
   {
      this.minCoPOffset = minCoPOffset;
      this.maxCoPOffset = maxCoPOffset;
   }

   public CoPPointName getCopPointName()
   {
      return copPointName;
   }

   public boolean getIsConstrainedToMinMax()
   {
      return isConstrainedToMinMax;
   }

   public boolean getIsConstrainedToSupportPolygon()
   {
      return isConstrainedToSupportPolygon;
   }

   public CoPSupportPolygonNames getStepLengthOffsetPolygon()
   {
      return stepLengthOffsetPolygon;
   }

   public double getStepLengthToCoPOffsetFactor()
   {
      return stepLengthToCoPOffsetFactor;
   }

   public Vector2D getCopOffsetsInFootFrame()
   {
      return copOffsetsInFootFrame;
   }

   public CoPSupportPolygonNames getSupportPolygonName()
   {
      return supportPolygonName;
   }

   public Vector2D getCopOffsetBoundsInFootFrame()
   {
      return copOffsetBoundsInFootFrame;
   }

   public YoFrameVector2d getCoPOffsets(RobotSide robotSide)
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
