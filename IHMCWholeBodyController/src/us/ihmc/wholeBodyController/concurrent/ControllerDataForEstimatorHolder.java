package us.ihmc.wholeBodyController.concurrent;

import javax.vecmath.Point3d;

import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;

public class ControllerDataForEstimatorHolder
{
   // Do not use FramePoint here, as ReferenceFrames are not shared between controller/estimator
   private final SideDependentList<Point3d> centerOfPressure = new SideDependentList<>();
   
   public ControllerDataForEstimatorHolder()
   {
      for(RobotSide robotSide : RobotSide.values)
      {
         centerOfPressure.put(robotSide, new Point3d());
      }
   }
   
   public void setCenterOfPressureInSoleFrame(RobotSide robotSide, Point3d point)
   {
      centerOfPressure.get(robotSide).set(point);
   }
   
   public void packCenterOfPressureInSoleFrame(Point3d pointToPack, RobotSide robotSide)
   {
      pointToPack.set(centerOfPressure.get(robotSide));
   }
   
   public static class Builder implements us.ihmc.concurrent.Builder<ControllerDataForEstimatorHolder>
   {

      @Override
      public ControllerDataForEstimatorHolder newInstance()
      {
         return new ControllerDataForEstimatorHolder();
      }
      
   }
}
