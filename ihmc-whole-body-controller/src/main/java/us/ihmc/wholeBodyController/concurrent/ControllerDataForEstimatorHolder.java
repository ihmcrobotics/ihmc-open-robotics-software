package us.ihmc.wholeBodyController.concurrent;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.IntermediateDesiredJointDataHolder;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;

public class ControllerDataForEstimatorHolder
{
   // Do not use FramePoint here, as HumanoidReferenceFrames are not shared between controller/estimator
   private final List<Point2D> centerOfPressure = new ArrayList<>();
   private AtomicReference<RobotMotionStatus> robotMotionStatus = new AtomicReference<RobotMotionStatus>(null);

   private final CenterOfPressureDataHolder controllerCenterOfPressureDataHolder;
   private final CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder;

   private final RobotMotionStatusHolder controllerRobotMotionStatusHolder;
   private final RobotMotionStatusHolder estimatorRobotMotionStatusHolder;

   private final IntermediateDesiredJointDataHolder intermediateDesiredJointDataHolder;

   private final List<ReferenceFrame> estimatorSoleFrames;
   private final List<ReferenceFrame> controllerSoleFrames;

   public ControllerDataForEstimatorHolder(CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder,
                                           CenterOfPressureDataHolder controllerCenterOfPressureDataHolder,
                                           RobotMotionStatusHolder estimatorRobotMotionStatusHolder, RobotMotionStatusHolder controllerRobotMotionStatusHolder,
                                           JointDesiredOutputList estimatorJointDataHolder, JointDesiredOutputList controllerJointDataHolder,
                                           List<ReferenceFrame> estimatorSoleFrames, List<ReferenceFrame> controllerSoleFrames)
   {
      this.estimatorCenterOfPressureDataHolder = estimatorCenterOfPressureDataHolder;
      this.controllerCenterOfPressureDataHolder = controllerCenterOfPressureDataHolder;

      this.estimatorRobotMotionStatusHolder = estimatorRobotMotionStatusHolder;
      this.controllerRobotMotionStatusHolder = controllerRobotMotionStatusHolder;

      this.intermediateDesiredJointDataHolder = new IntermediateDesiredJointDataHolder(estimatorJointDataHolder, controllerJointDataHolder);

      this.estimatorSoleFrames = estimatorSoleFrames;
      this.controllerSoleFrames = controllerSoleFrames;

      for (int i = 0; i < estimatorSoleFrames.size(); i++)
      {
         centerOfPressure.add(new Point2D());
      }
   }

   public void readControllerDataIntoEstimator()
   {
      for (int i = 0; i < estimatorSoleFrames.size(); i++)
      {
         estimatorCenterOfPressureDataHolder.setCenterOfPressure(estimatorSoleFrames.get(i), centerOfPressure.get(i), i);
      }

      if (robotMotionStatus.get() != null)
         estimatorRobotMotionStatusHolder.setCurrentRobotMotionStatus(robotMotionStatus.getAndSet(null));
      
      intermediateDesiredJointDataHolder.readIntoEstimator();
   }

   public void writeControllerDataFromController()
   {
      for (int i = 0; i < controllerSoleFrames.size(); i++)
      {
         centerOfPressure.get(i).set(controllerCenterOfPressureDataHolder.getCenterOfPressure(i));
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

      private final JointDesiredOutputList estimatorDesiredJointDataHolder;
      private final JointDesiredOutputList controllerDesiredJointDataHolder;

      private final List<ReferenceFrame> estimatorSoleFrames;
      private final List<ReferenceFrame> controllerSoleFrames;

      public Builder(CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder, CenterOfPressureDataHolder controllerCenterOfPressureDataHolder,
            RobotMotionStatusHolder estimatorRobotMotionStatusHolder, RobotMotionStatusHolder controllerRobotMotionStatusHolder,
            JointDesiredOutputList estimatorDesiredJointDataHolder, JointDesiredOutputList controllerDesiredJointDataHolder,
            List<ReferenceFrame> estimatorSoleFrames, List<ReferenceFrame> controllerSoleFrames)
      {
         this.estimatorCenterOfPressureDataHolder = estimatorCenterOfPressureDataHolder;
         this.controllerCenterOfPressureDataHolder = controllerCenterOfPressureDataHolder;

         this.estimatorRobotMotionStatusHolder = estimatorRobotMotionStatusHolder;
         this.controllerRobotMotionStatusHolder = controllerRobotMotionStatusHolder;

         this.estimatorDesiredJointDataHolder = estimatorDesiredJointDataHolder;
         this.controllerDesiredJointDataHolder = controllerDesiredJointDataHolder;

         this.estimatorSoleFrames = estimatorSoleFrames;
         this.controllerSoleFrames = controllerSoleFrames;
      }

      @Override
      public ControllerDataForEstimatorHolder newInstance()
      {
         return new ControllerDataForEstimatorHolder(estimatorCenterOfPressureDataHolder, controllerCenterOfPressureDataHolder,
                                                     estimatorRobotMotionStatusHolder, controllerRobotMotionStatusHolder, estimatorDesiredJointDataHolder,
                                                     controllerDesiredJointDataHolder, estimatorSoleFrames, controllerSoleFrames);
      }

   }
}
