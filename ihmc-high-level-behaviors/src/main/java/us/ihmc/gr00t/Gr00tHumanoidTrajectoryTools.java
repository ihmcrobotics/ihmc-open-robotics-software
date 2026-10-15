package us.ihmc.gr00t;

import controller_msgs.NeckTrajectoryMessage;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;

/** Pure trajectory, convergence, and controller-message helpers. */
final class Gr00tHumanoidTrajectoryTools
{
   private Gr00tHumanoidTrajectoryTools()
   {
   }

   static double computeTrajectoryDuration(Gr00tHumanoidConfiguration configuration,
                                           boolean initialTransit,
                                           double pathLength,
                                           double angularTravel)
   {
      double minimum = initialTransit ? configuration.initialTrajectoryTime() : configuration.trajectoryTime();
      return Math.max(minimum,
                      Math.max(pathLength / configuration.maxLinearVelocity(), angularTravel / configuration.maxAngularVelocity()));
   }

   static double[] computeTrajectoryTimes(Gr00tHumanoidConfiguration configuration,
                                          FramePose3D[] cartesianTargets,
                                          boolean initialTransit)
   {
      double[] segmentDurations = new double[cartesianTargets.length];
      double unscaledDuration = 0.0;
      for (int sample = 1; sample < cartesianTargets.length; sample++)
      {
         double linearTravel = cartesianTargets[sample].getPosition().distance(cartesianTargets[sample - 1].getPosition());
         double angularTravel = cartesianTargets[sample].getOrientation().distance(cartesianTargets[sample - 1].getOrientation());
         segmentDurations[sample] = Math.max(configuration.policyActionPeriod(),
                                             Math.max(linearTravel / configuration.maxLinearVelocity(),
                                                      angularTravel / configuration.maxAngularVelocity()));
         unscaledDuration += segmentDurations[sample];
      }

      double minimumDuration = initialTransit ? configuration.initialTrajectoryTime() : configuration.trajectoryTime();
      double scale = unscaledDuration > 0.0 ? Math.max(1.0, minimumDuration / unscaledDuration) : 1.0;
      double[] trajectoryTimes = new double[cartesianTargets.length];
      for (int sample = 1; sample < trajectoryTimes.length; sample++)
         trajectoryTimes[sample] = trajectoryTimes[sample - 1] + scale * segmentDurations[sample];
      return trajectoryTimes;
   }

   static double stageHoldRemainingSeconds(Gr00tHumanoidConfiguration configuration,
                                           double elapsedSeconds,
                                           double trajectoryDurationSeconds)
   {
      return Math.max(0.0, trajectoryDurationSeconds + configuration.stageSettleTime() - elapsedSeconds);
   }

   static double endpointConfirmationRemainingSeconds(Gr00tHumanoidConfiguration configuration,
                                                      double elapsedSeconds,
                                                      double trajectoryDurationSeconds)
   {
      return Math.max(0.0,
                      trajectoryDurationSeconds + configuration.stageSettleTime()
                      + configuration.endpointConvergenceGraceTime() - elapsedSeconds);
   }

   static boolean isActionChunkComplete(Gr00tHumanoidConfiguration configuration,
                                        double elapsedSeconds,
                                        double trajectoryDurationSeconds,
                                        boolean endpointConverged)
   {
      if (stageHoldRemainingSeconds(configuration, elapsedSeconds, trajectoryDurationSeconds) > 0.0)
         return false;
      return endpointConverged || endpointConfirmationRemainingSeconds(configuration, elapsedSeconds, trajectoryDurationSeconds) <= 0.0;
   }

   /** Returns the latest action waypoint whose timestamp has elapsed; zero means none is due. */
   static int latestDueAction(double[] trajectoryTimes, double elapsedSeconds)
   {
      int dueAction = 0;
      while (dueAction + 1 < trajectoryTimes.length && trajectoryTimes[dueAction + 1] <= elapsedSeconds)
         dueAction++;
      return dueAction;
   }

   static boolean isEndpointConverged(Gr00tHumanoidConfiguration configuration,
                                      Pose3DReadOnly currentPose,
                                      Pose3DReadOnly predictedEndpoint)
   {
      return currentPose.getPosition().distance(predictedEndpoint.getPosition()) <= configuration.endpointPositionThreshold()
             && currentPose.getOrientation().distance(predictedEndpoint.getOrientation()) <= configuration.endpointOrientationThreshold();
   }

   static NeckTrajectoryMessage createNeckTrajectoryMessage(double[] trajectoryTimes, double[][] neckJointPositions)
   {
      int pointCount = trajectoryTimes.length - 1;
      if (pointCount < 1 || neckJointPositions.length != trajectoryTimes.length)
         throw new IllegalArgumentException("Neck trajectory positions must match at least two timestamps");

      NeckTrajectoryMessage message = HumanoidMessageTools.createNeckTrajectoryMessage(trajectoryTimes[pointCount],
                                                                                        neckJointPositions[pointCount]);
      for (int joint = 0; joint < 2; joint++)
      {
         var jointTrajectory = message.getJointspaceTrajectory().getJointTrajectoryMessages().get(joint);
         jointTrajectory.getTrajectoryPoints().clear();
         for (int sample = 1; sample <= pointCount; sample++)
         {
            var point = jointTrajectory.getTrajectoryPoints().add();
            point.setTime(trajectoryTimes[sample]);
            point.setPosition(neckJointPositions[sample][joint]);
            double velocity = sample == pointCount ? 0.0
                                                   : (neckJointPositions[sample + 1][joint] - neckJointPositions[sample - 1][joint])
                                                     / (trajectoryTimes[sample + 1] - trajectoryTimes[sample - 1]);
            point.setVelocity(velocity);
         }
      }
      return message;
   }

}
