package us.ihmc.humanoidRobotics.model;

import javax.vecmath.Point2d;

import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class CenterOfPressureDataHolder
{
   private final SideDependentList<ReferenceFrame> soleFrames;
   private final SideDependentList<Point2d> centerOfPressures = new SideDependentList<>(new Point2d(), new Point2d());

   public CenterOfPressureDataHolder(SideDependentList<ReferenceFrame> soleFrames)
   {
      this.soleFrames = soleFrames;
   }

   public void setCenterOfPressure(Point2d centerOfPressure, RobotSide robotSide)
   {
      centerOfPressures.get(robotSide).set(centerOfPressure);
   }

   public void setCenterOfPressure(FramePoint2d centerOfPressure, RobotSide robotSide)
   {
      if (centerOfPressure != null)
      {
         centerOfPressure.checkReferenceFrameMatch(soleFrames.get(robotSide));
         centerOfPressure.get(centerOfPressures.get(robotSide));
      }
      else
      {
         centerOfPressures.get(robotSide).set(Double.NaN, Double.NaN);
      }
   }

   public void getCenterOfPressure(Point2d centerOfPressureToPack, RobotSide robotSide)
   {
      centerOfPressureToPack.set(centerOfPressures.get(robotSide));
   }
   
   public void getCenterOfPressure(FramePoint2d centerOfPressureToPack, RobotSide robotSide)
   {
      centerOfPressureToPack.setIncludingFrame(soleFrames.get(robotSide), centerOfPressures.get(robotSide));
   }
}
