package us.ihmc.wholeBodyController.concurrent;

import javax.vecmath.Point2d;

import us.ihmc.utilities.humanoidRobot.model.CenterOfPressureDataHolder;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;

public class ControllerDataForEstimatorHolder
{
   // Do not use FramePoint here, as ReferenceFrames are not shared between controller/estimator
   private final SideDependentList<Point2d> centerOfPressure = new SideDependentList<>();

   private final CenterOfPressureDataHolder controllerCenterOfPressureDataHolder;
   private final CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder;
   
   public ControllerDataForEstimatorHolder(CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder, CenterOfPressureDataHolder controllerCenterOfPressureDataHolder)
   {
      for(RobotSide robotSide : RobotSide.values)
      {
         centerOfPressure.put(robotSide, new Point2d());
      }

      this.controllerCenterOfPressureDataHolder = controllerCenterOfPressureDataHolder;
      this.estimatorCenterOfPressureDataHolder = estimatorCenterOfPressureDataHolder;
   }
   
   public void setCenterOfPressureInSoleFrame()
   {
      for(RobotSide robotSide : RobotSide.values)
      {
         controllerCenterOfPressureDataHolder.getCenterOfPressure(centerOfPressure.get(robotSide), robotSide);
      }
   }
   
   public void getCenterOfPressureInSoleFrame()
   {
      for(RobotSide robotSide : RobotSide.values)
      {
         estimatorCenterOfPressureDataHolder.setCenterOfPressure(centerOfPressure.get(robotSide), robotSide);
      }
   }
   
   public static class Builder implements us.ihmc.concurrent.Builder<ControllerDataForEstimatorHolder>
   {
      private final CenterOfPressureDataHolder controllerCenterOfPressureDataHolder;
      private final CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder;

      public Builder(CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder, CenterOfPressureDataHolder controllerCenterOfPressureDataHolder)
      {
         this.controllerCenterOfPressureDataHolder = controllerCenterOfPressureDataHolder;
         this.estimatorCenterOfPressureDataHolder = estimatorCenterOfPressureDataHolder;
      }

      @Override
      public ControllerDataForEstimatorHolder newInstance()
      {
         return new ControllerDataForEstimatorHolder(estimatorCenterOfPressureDataHolder, controllerCenterOfPressureDataHolder);
      }
      
   }
}
