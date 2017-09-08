package us.ihmc.wholeBodyController.concurrent;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.IntermediateDesiredJointDataHolder;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderList;

public class ControllerDataForEstimatorHolder
{
   // Do not use FramePoint here, as HumanoidReferenceFrames are not shared between controller/estimator
   private final Map<String, Point2D> centerOfPressure = new HashMap<String, Point2D>();
   private AtomicReference<RobotMotionStatus> robotMotionStatus = new AtomicReference<RobotMotionStatus>(null);

   private final List<RigidBody> controllerFeet;
   private final List<RigidBody> estimatorFeet;
   private final CenterOfPressureDataHolder controllerCenterOfPressureDataHolder;
   private final CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder;

   private final RobotMotionStatusHolder controllerRobotMotionStatusHolder;
   private final RobotMotionStatusHolder estimatorRobotMotionStatusHolder;

   private final IntermediateDesiredJointDataHolder intermediateDesiredJointDataHolder;

   public ControllerDataForEstimatorHolder(CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder,
         CenterOfPressureDataHolder controllerCenterOfPressureDataHolder, RobotMotionStatusHolder estimatorRobotMotionStatusHolder,
         RobotMotionStatusHolder controllerRobotMotionStatusHolder, LowLevelOneDoFJointDesiredDataHolderList estimatorJointDataHolder,
         LowLevelOneDoFJointDesiredDataHolderList controllerJointDataHolder)
   {      
      this.controllerFeet = new ArrayList<>(controllerCenterOfPressureDataHolder.getRigidBodies());
      this.estimatorFeet = new ArrayList<>(estimatorCenterOfPressureDataHolder.getRigidBodies());

      this.estimatorCenterOfPressureDataHolder = estimatorCenterOfPressureDataHolder;
      this.controllerCenterOfPressureDataHolder = controllerCenterOfPressureDataHolder;

      this.estimatorRobotMotionStatusHolder = estimatorRobotMotionStatusHolder;
      this.controllerRobotMotionStatusHolder = controllerRobotMotionStatusHolder;

      this.intermediateDesiredJointDataHolder = new IntermediateDesiredJointDataHolder(estimatorJointDataHolder, controllerJointDataHolder);
      
      for (int i = 0; i < controllerFeet.size(); i++)
      {
         RigidBody foot = controllerFeet.get(i);
         centerOfPressure.put(foot.getName(), new Point2D());
      }
   }

   public void readControllerDataIntoEstimator()
   {
      for (int i = 0; i < estimatorFeet.size(); i++)
      {
         RigidBody foot = estimatorFeet.get(i);
         estimatorCenterOfPressureDataHolder.setCenterOfPressure(centerOfPressure.get(foot.getName()), foot);
      }

      if (robotMotionStatus.get() != null)
         estimatorRobotMotionStatusHolder.setCurrentRobotMotionStatus(robotMotionStatus.getAndSet(null));
      
      intermediateDesiredJointDataHolder.readIntoEstimator();
   }

   public void writeControllerDataFromController()
   {
      for (int i = 0; i < controllerFeet.size(); i++)
      {
         RigidBody foot = controllerFeet.get(i);
         controllerCenterOfPressureDataHolder.getCenterOfPressureByName(centerOfPressure.get(foot.getName()), foot);
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

      private final LowLevelOneDoFJointDesiredDataHolderList estimatorDesiredJointDataHolder;
      private final LowLevelOneDoFJointDesiredDataHolderList controllerDesiredJointDataHolder;

      public Builder(CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder, CenterOfPressureDataHolder controllerCenterOfPressureDataHolder,
            RobotMotionStatusHolder estimatorRobotMotionStatusHolder, RobotMotionStatusHolder controllerRobotMotionStatusHolder,
            LowLevelOneDoFJointDesiredDataHolderList estimatorDesiredJointDataHolder, LowLevelOneDoFJointDesiredDataHolderList controllerDesiredJointDataHolder)
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
