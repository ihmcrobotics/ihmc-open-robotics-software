package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class WholeBodyTrajectoryToolboxMessageTools
{
   public static interface FunctionTrajectory
   {
      public Pose3D compute(double time);
   }

   public static FunctionTrajectory createFunctionTrajectory(WaypointBasedTrajectoryMessage message)
   {
      return new FunctionTrajectory()
      {
         @Override
         public Pose3D compute(double time)
         {
            Pose3D current = new Pose3D();

            Pose3D previous = null;
            Pose3D next = null;
            double t0 = Double.NaN;
            double tf = Double.NaN;

            for (int i = 1; i < message.getNumberOfWaypoints(); i++)
            {
               t0 = message.getWaypointTime(i - 1);
               tf = message.getWaypointTime(i);
               previous = message.getWaypoint(i - 1);
               next = message.getWaypoint(i);
               if (time < message.getWaypointTime(i))
                  break;
            }

            double alpha = (time - t0) / (tf - t0);
            alpha = MathTools.clamp(alpha, 0.0, 1.0);
            current.interpolate(previous, next, alpha);

            return current;
         }
      };
   }

   public static WaypointBasedTrajectoryMessage createTrajectoryMessage(RigidBody endEffector, double t0, double tf, double timeResolution,
                                                                        FunctionTrajectory trajectoryToDiscretize, SelectionMatrix6D selectionMatrix)
   {
      int numberOfWaypoints = (int) Math.round((tf - t0) / timeResolution) + 1;
      // Adjust the timeResolution using the numberOfWaypoints:
      timeResolution = (tf - t0) / (numberOfWaypoints - 1);

      double[] waypointTimes = new double[numberOfWaypoints];
      Pose3D[] waypoints = new Pose3D[numberOfWaypoints];

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double waypointTime = i * timeResolution + t0;

         waypointTimes[i] = waypointTime;
         waypoints[i] = trajectoryToDiscretize.compute(waypointTime);
      }

      return new WaypointBasedTrajectoryMessage(endEffector, waypointTimes, waypoints, selectionMatrix);
   }

   public static double[] createDefaultExplorationAmplitudeArray(ConfigurationSpaceName... configurationSpaceNames)
   {
      double[] lowerLimit = new double[configurationSpaceNames.length];
      for (int i = 0; i < configurationSpaceNames.length; i++)
         lowerLimit[i] = configurationSpaceNames[i].getDefaultExplorationAmplitude();
      return lowerLimit;
   }

   public static double[] createDefaultExplorationUpperLimitArray(ConfigurationSpaceName... configurationSpaceNames)
   {
      double[] upperLimit = new double[configurationSpaceNames.length];
      for (int i = 0; i < configurationSpaceNames.length; i++)
         upperLimit[i] = configurationSpaceNames[i].getDefaultExplorationUpperLimit();
      return upperLimit;
   }

   public static double[] createDefaultExplorationLowerLimitArray(ConfigurationSpaceName... configurationSpaceNames)
   {
      double[] lowerLimit = new double[configurationSpaceNames.length];
      for (int i = 0; i < configurationSpaceNames.length; i++)
         lowerLimit[i] = configurationSpaceNames[i].getDefaultExplorationLowerLimit();
      return lowerLimit;
   }

   public static double computePoseDistance(Pose3D poseOne, Pose3D poseTwo, double positionWeight, double orientationWeight)
   {
      double distance = 0.0;

      double positionDistance = poseOne.getPositionDistance(poseTwo);
      double orientationDistance = poseOne.getOrientationDistance(poseTwo);
      orientationDistance = AngleTools.trimAngleMinusPiToPi(orientationDistance);
      orientationDistance = Math.abs(orientationDistance);
      distance = positionWeight * positionDistance + orientationWeight * orientationDistance;

      return distance;
   }
}
