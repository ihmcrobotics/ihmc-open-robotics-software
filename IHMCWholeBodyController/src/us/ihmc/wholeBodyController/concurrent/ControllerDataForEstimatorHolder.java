package us.ihmc.wholeBodyController.concurrent;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point2d;

import us.ihmc.utilities.humanoidRobot.RobotMotionStatus;
import us.ihmc.utilities.humanoidRobot.model.CenterOfPressureDataHolder;
import us.ihmc.utilities.humanoidRobot.model.RobotMotionStatusHolder;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;

public class ControllerDataForEstimatorHolder
{
   // Do not use FramePoint here, as ReferenceFrames are not shared between controller/estimator
   private final SideDependentList<Point2d> centerOfPressure = new SideDependentList<>();
   private AtomicReference<RobotMotionStatus> robotMotionStatus = new AtomicReference<RobotMotionStatus>(null);

   private final CenterOfPressureDataHolder controllerCenterOfPressureDataHolder;
   private final CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder;

   private final RobotMotionStatusHolder controllerRobotMotionStatusHolder;
   private final RobotMotionStatusHolder estimatorRobotMotionStatusHolder;

   public ControllerDataForEstimatorHolder(CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder,
         CenterOfPressureDataHolder controllerCenterOfPressureDataHolder, RobotMotionStatusHolder estimatorRobotMotionStatusHolder,
         RobotMotionStatusHolder controllerRobotMotionStatusHolder)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         centerOfPressure.put(robotSide, new Point2d());
      }

      this.estimatorCenterOfPressureDataHolder = estimatorCenterOfPressureDataHolder;
      this.controllerCenterOfPressureDataHolder = controllerCenterOfPressureDataHolder;

      this.estimatorRobotMotionStatusHolder = estimatorRobotMotionStatusHolder;
      this.controllerRobotMotionStatusHolder = controllerRobotMotionStatusHolder;
   }

   public void copyControllerData()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         controllerCenterOfPressureDataHolder.getCenterOfPressure(centerOfPressure.get(robotSide), robotSide);
      }
      robotMotionStatus.set(controllerRobotMotionStatusHolder.getCurrentRobotMotionStatus());
   }

   public void parseControllerDataToEstimator()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         estimatorCenterOfPressureDataHolder.setCenterOfPressure(centerOfPressure.get(robotSide), robotSide);
      }

      if (robotMotionStatus.get() != null)
         estimatorRobotMotionStatusHolder.setCurrentRobotMotionStatus(robotMotionStatus.getAndSet(null));
   }

   public static class Builder implements us.ihmc.concurrent.Builder<ControllerDataForEstimatorHolder>
   {
      private final CenterOfPressureDataHolder controllerCenterOfPressureDataHolder;
      private final CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder;

      private final RobotMotionStatusHolder controllerRobotMotionStatusHolder;
      private final RobotMotionStatusHolder estimatorRobotMotionStatusHolder;

      public Builder(CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder, CenterOfPressureDataHolder controllerCenterOfPressureDataHolder,
            RobotMotionStatusHolder estimatorRobotMotionStatusHolder, RobotMotionStatusHolder controllerRobotMotionStatusHolder)
      {
         this.estimatorCenterOfPressureDataHolder = estimatorCenterOfPressureDataHolder;
         this.controllerCenterOfPressureDataHolder = controllerCenterOfPressureDataHolder;

         this.estimatorRobotMotionStatusHolder = estimatorRobotMotionStatusHolder;
         this.controllerRobotMotionStatusHolder = controllerRobotMotionStatusHolder;
      }

      @Override
      public ControllerDataForEstimatorHolder newInstance()
      {
         return new ControllerDataForEstimatorHolder(estimatorCenterOfPressureDataHolder, controllerCenterOfPressureDataHolder, estimatorRobotMotionStatusHolder, controllerRobotMotionStatusHolder);
      }

   }
}
