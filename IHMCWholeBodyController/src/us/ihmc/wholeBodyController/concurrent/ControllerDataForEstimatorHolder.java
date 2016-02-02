package us.ihmc.wholeBodyController.concurrent;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point2d;

import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.humanoidRobotics.model.IntermediateDesiredJointDataHolder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.model.DesiredJointDataHolder;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;

public class ControllerDataForEstimatorHolder
{
   // Do not use FramePoint here, as HumanoidReferenceFrames are not shared between controller/estimator
   private final SideDependentList<Point2d> centerOfPressure = new SideDependentList<>();
   private AtomicReference<RobotMotionStatus> robotMotionStatus = new AtomicReference<RobotMotionStatus>(null);

   private final List<RigidBody> feet;
   private final CenterOfPressureDataHolder controllerCenterOfPressureDataHolder;
   private final CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder;

   private final RobotMotionStatusHolder controllerRobotMotionStatusHolder;
   private final RobotMotionStatusHolder estimatorRobotMotionStatusHolder;

   private final IntermediateDesiredJointDataHolder intermediateDesiredJointDataHolder;

   public ControllerDataForEstimatorHolder(CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder,
         CenterOfPressureDataHolder controllerCenterOfPressureDataHolder, RobotMotionStatusHolder estimatorRobotMotionStatusHolder,
         RobotMotionStatusHolder controllerRobotMotionStatusHolder, DesiredJointDataHolder estimatorJointDataHolder,
         DesiredJointDataHolder controllerJointDataHolder)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         centerOfPressure.put(robotSide, new Point2d());
      }
      
      this.feet = controllerCenterOfPressureDataHolder.getRigidBodies();

      this.estimatorCenterOfPressureDataHolder = estimatorCenterOfPressureDataHolder;
      this.controllerCenterOfPressureDataHolder = controllerCenterOfPressureDataHolder;

      this.estimatorRobotMotionStatusHolder = estimatorRobotMotionStatusHolder;
      this.controllerRobotMotionStatusHolder = controllerRobotMotionStatusHolder;

      this.intermediateDesiredJointDataHolder = new IntermediateDesiredJointDataHolder(estimatorJointDataHolder, controllerJointDataHolder);
   }

   public void readControllerDataIntoEstimator()
   {
      for (RigidBody foot : feet)
      {
         estimatorCenterOfPressureDataHolder.setCenterOfPressure(centerOfPressure.get(foot), foot);
      }

      if (robotMotionStatus.get() != null)
         estimatorRobotMotionStatusHolder.setCurrentRobotMotionStatus(robotMotionStatus.getAndSet(null));
      
      intermediateDesiredJointDataHolder.readIntoEstimator();
   }

   public void writeControllerDataFromController()
   {
      for (RigidBody foot : feet)
      {
         controllerCenterOfPressureDataHolder.getCenterOfPressure(centerOfPressure.get(foot), foot);
      }

      robotMotionStatus.set(controllerRobotMotionStatusHolder.getCurrentRobotMotionStatus());

      intermediateDesiredJointDataHolder.copyFromController();
   }

   public static class Builder implements us.ihmc.concurrent.Builder<ControllerDataForEstimatorHolder>
   {
      private final CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder;
      private final CenterOfPressureDataHolder controllerCenterOfPressureDataHolder;

      private final RobotMotionStatusHolder estimatorRobotMotionStatusHolder;
      private final RobotMotionStatusHolder controllerRobotMotionStatusHolder;

      private final DesiredJointDataHolder estimatorDesiredJointDataHolder;
      private final DesiredJointDataHolder controllerDesiredJointDataHolder;

      public Builder(CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder, CenterOfPressureDataHolder controllerCenterOfPressureDataHolder,
            RobotMotionStatusHolder estimatorRobotMotionStatusHolder, RobotMotionStatusHolder controllerRobotMotionStatusHolder,
            DesiredJointDataHolder estimatorDesiredJointDataHolder, DesiredJointDataHolder controllerDesiredJointDataHolder)
      {
         this.estimatorCenterOfPressureDataHolder = estimatorCenterOfPressureDataHolder;
         this.controllerCenterOfPressureDataHolder = controllerCenterOfPressureDataHolder;

         this.estimatorRobotMotionStatusHolder = estimatorRobotMotionStatusHolder;
         this.controllerRobotMotionStatusHolder = controllerRobotMotionStatusHolder;

         this.estimatorDesiredJointDataHolder = estimatorDesiredJointDataHolder;
         this.controllerDesiredJointDataHolder = controllerDesiredJointDataHolder;
      }

      @Override
      public ControllerDataForEstimatorHolder newInstance()
      {
         return new ControllerDataForEstimatorHolder(estimatorCenterOfPressureDataHolder, controllerCenterOfPressureDataHolder,
               estimatorRobotMotionStatusHolder, controllerRobotMotionStatusHolder, estimatorDesiredJointDataHolder, controllerDesiredJointDataHolder);
      }

   }
}
