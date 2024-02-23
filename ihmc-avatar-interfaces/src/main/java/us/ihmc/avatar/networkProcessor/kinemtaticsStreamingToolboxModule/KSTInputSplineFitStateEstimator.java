package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.robotics.math.filters.SplineBasedOnlinePositionFilter3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

public class KSTInputSplineFitStateEstimator implements KSTInputStateEstimator
{

   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble lastLocalTime = new YoDouble("lastLocalTime", registry);

   private final Map<RigidBodyReadOnly, SingleEndEffectorStateEstimator> inputPoseStateEstimators = new HashMap<>();
   private final SingleEndEffectorStateEstimator[] inputPoseStateEstimatorsArray;

   public KSTInputSplineFitStateEstimator(Collection<? extends RigidBodyBasics> endEffectors, YoRegistry parentRegistry)
   {
      int windowSize = 10;

      for (RigidBodyReadOnly endEffector : endEffectors)
      {
         inputPoseStateEstimators.put(endEffector, new SingleEndEffectorStateEstimator(endEffector, windowSize, registry));
      }
      inputPoseStateEstimatorsArray = inputPoseStateEstimators.values().toArray(new SingleEndEffectorStateEstimator[0]);

      parentRegistry.addChild(registry);
   }

   @Override
   public void reset()
   {
      for (SingleEndEffectorStateEstimator inputPoseEstimator : inputPoseStateEstimatorsArray)
         inputPoseEstimator.reset();
   }

   @Override
   public void update(double timeLocal,
                      boolean isNewInput,
                      KinematicsStreamingToolboxInputCommand latestInputCommand,
                      KinematicsStreamingToolboxInputCommand previousRawInputCommand)
   {
      if (isNewInput)
      {
         lastLocalTime.set(timeLocal);

         for (int i = 0; i < latestInputCommand.getNumberOfInputs(); i++)
         {
            KinematicsToolboxRigidBodyCommand input = latestInputCommand.getInput(i);

            SingleEndEffectorStateEstimator inputPoseEstimator = inputPoseStateEstimators.get(input.getEndEffector());

            if (inputPoseEstimator == null)
               continue;

            FramePose3D desiredPose = input.getDesiredPose();
            inputPoseEstimator.update(timeLocal, desiredPose);
            desiredPose.getPosition().set(inputPoseEstimator.positionFilter.getEstimatedPosition());
         }
      }
      else
      {
         double timeToExtrapolateFor = timeLocal - lastLocalTime.getDoubleValue();

         for (int i = 0; i < latestInputCommand.getNumberOfInputs(); i++)
         {
            KinematicsToolboxRigidBodyCommand input = latestInputCommand.getInput(i);

            SingleEndEffectorStateEstimator inputPoseEstimator = inputPoseStateEstimators.get(input.getEndEffector());

            if (inputPoseEstimator == null)
               continue;

            FramePose3D desiredPose = input.getDesiredPose();
            inputPoseEstimator.extrapolate(timeToExtrapolateFor);
            desiredPose.getPosition().set(inputPoseEstimator.positionFilter.getEstimatedPosition());
         }
      }
   }

   private static class SingleEndEffectorStateEstimator
   {
      private final YoFramePose3D rawInputPose;
      private final SplineBasedOnlinePositionFilter3D positionFilter;

      public SingleEndEffectorStateEstimator(RigidBodyReadOnly endEffector, int windowSize, YoRegistry registry)
      {
         rawInputPose = new YoFramePose3D(endEffector.getName() + "RawInputPose", worldFrame, registry);
         positionFilter = new SplineBasedOnlinePositionFilter3D(endEffector.getName(), windowSize, Double.POSITIVE_INFINITY, 3, registry);
      }

      public void reset()
      {
         positionFilter.reset();
      }

      public void update(double timeSource, Pose3DReadOnly pose)
      {
         rawInputPose.set(pose);
         positionFilter.update(timeSource, pose.getPosition());
      }

      public void extrapolate(double timeToExtrapolateFor)
      {
         positionFilter.compute(positionFilter.getNewestPointTime() + timeToExtrapolateFor);
      }
   }
}
