package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.robotics.math.trajectories.generators.TrajectoryPointOptimizer;

/**
 * The input of this calculator are a sequence of {@link KinematicsToolboxOutputStatus} and key frame times.
 * <p>
 * The calculator will optimize way point velocities to minimize square of acceleration for the entire trajectory.
 * <p>
 * And also it is able to optimize optimal way point times.
 * <p>
 * {@code WholeBodyTrajectoryPointCalculator} will return joint velocity bounds and the bounds will be used in {@link KinematicsPlanningToolboxController}.
 * <p>
 * Finally, the optimized key frame times and joint velocities will be packed on {@link KinematicsPlanningToolboxOutputStatus}.
 */
public class WholeBodyTrajectoryPointCalculator
{
   private final List<KinematicsToolboxOutputStatus> keyFrames = new ArrayList<KinematicsToolboxOutputStatus>();
   private final TDoubleArrayList keyFrameTimes = new TDoubleArrayList();
   private TDoubleArrayList normalizedWaypointTimes = new TDoubleArrayList();

   private final TrajectoryPointOptimizer trajectoryPointOptimizer;
   private static final int numberOfIterationsToOptimizeWaypointTimes = 100;
   private static final double initialJointVelocity = 0.0;
   private static final double finalJointVelocity = 0.0;

   private final List<String> jointNames = new ArrayList<String>();
   private final Map<String, OneDoFJointTrajectoryHolder> jointNameToTrajectoryHolderMap = new HashMap<>();

   private final KinematicsToolboxOutputConverter converter;
   private final boolean SAVE_TRAJECTORY_DATA = false;

   public WholeBodyTrajectoryPointCalculator(DRCRobotModel drcRobotModel)
   {
      OneDoFJointBasics[] allJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(drcRobotModel.createFullRobotModel());
      int numberOfWholeBodyJoints = allJointsExcludingHands.length;

      trajectoryPointOptimizer = new TrajectoryPointOptimizer(numberOfWholeBodyJoints);
      converter = new KinematicsToolboxOutputConverter(drcRobotModel);

      for (int i = 0; i < numberOfWholeBodyJoints; i++)
         jointNames.add(allJointsExcludingHands[i].getName());

      for (String jointName : jointNames)
         jointNameToTrajectoryHolderMap.put(jointName, new OneDoFJointTrajectoryHolder());
   }

   public void clear()
   {
      keyFrames.clear();
      keyFrameTimes.clear();
      for (String jointName : jointNames)
         jointNameToTrajectoryHolderMap.get(jointName).clear();
   }

   public void addKeyFrames(List<KinematicsToolboxOutputStatus> solutionKeyFrames, TDoubleArrayList solutionKeyFrameTimes)
   {
      keyFrames.addAll(solutionKeyFrames);
      keyFrameTimes.addAll(solutionKeyFrameTimes);
   }

   public void initializeCalculator()
   {
      for (KinematicsToolboxOutputStatus keyFrame : keyFrames)
      {
         converter.updateFullRobotModel(keyFrame);

         for (String jointName : jointNames)
         {
            double jointPosition = converter.getFullRobotModel().getOneDoFJointByName(jointName).getQ();
            jointNameToTrajectoryHolderMap.get(jointName).appendTrajectoryPoint(jointPosition);
         }
      }

      TDoubleArrayList firstWayPoints = new TDoubleArrayList();
      TDoubleArrayList lastWayPoints = new TDoubleArrayList();
      TDoubleArrayList firstWayPointVelocities = new TDoubleArrayList();
      TDoubleArrayList lastWayPointVelocities = new TDoubleArrayList();
      for (String jointName : jointNames)
      {
         firstWayPoints.add(jointNameToTrajectoryHolderMap.get(jointName).getFirstPoint());
         lastWayPoints.add(jointNameToTrajectoryHolderMap.get(jointName).getLastPoint());
         firstWayPointVelocities.add(initialJointVelocity);
         lastWayPointVelocities.add(finalJointVelocity);
      }

      int numberOfWayPoints = keyFrames.size() - 2;
      ArrayList<TDoubleArrayList> wayPointSets = new ArrayList<>();
      for (int i = 0; i < numberOfWayPoints; i++)
      {
         TDoubleArrayList wayPointsForAKeyFrameTime = new TDoubleArrayList();
         for (String jointName : jointNames)
         {
            double wayPoint = jointNameToTrajectoryHolderMap.get(jointName).getWayPoint(i);
            wayPointsForAKeyFrameTime.add(wayPoint);
         }
         wayPointSets.add(wayPointsForAKeyFrameTime);
      }

      trajectoryPointOptimizer.setEndPoints(firstWayPoints, firstWayPointVelocities, lastWayPoints, lastWayPointVelocities);
      trajectoryPointOptimizer.setWaypoints(wayPointSets);

      double lastKeyFrameTime = keyFrameTimes.get(keyFrameTimes.size() - 1);
      normalizedWaypointTimes.clear();
      for (int i = 1; i < keyFrameTimes.size() - 1; i++)
         normalizedWaypointTimes.add(keyFrameTimes.get(i) / lastKeyFrameTime);
   }

   public void computeOptimizingKeyFrameTimes()
   {
      trajectoryPointOptimizer.compute(numberOfIterationsToOptimizeWaypointTimes, normalizedWaypointTimes);
      updateOptimizedWaypointTimes();
      updateOptimizedJointVelocities();
   }

   public void computeForFixedKeyFrameTimes()
   {
      trajectoryPointOptimizer.computeForFixedTime(normalizedWaypointTimes);
      updateOptimizedJointVelocities();
   }

   private void updateOptimizedJointVelocities()
   {
      for (String jointName : jointNames)
         jointNameToTrajectoryHolderMap.get(jointName).addJointVelocity(initialJointVelocity);

      double lastKeyFrameTime = keyFrameTimes.get(keyFrameTimes.size() - 1);
      int numberOfWayPoints = keyFrames.size() - 2;
      TDoubleArrayList velocityToPack = new TDoubleArrayList();
      for (int i = 0; i < numberOfWayPoints; i++)
      {
         trajectoryPointOptimizer.getWaypointVelocity(velocityToPack, i);
         for (int j = 0; j < jointNames.size(); j++)
            jointNameToTrajectoryHolderMap.get(jointNames.get(j)).addJointVelocity(velocityToPack.get(j) / lastKeyFrameTime);
      }

      for (String jointName : jointNames)
         jointNameToTrajectoryHolderMap.get(jointName).addJointVelocity(finalJointVelocity);
   }

   private void updateOptimizedWaypointTimes()
   {
      double lastKeyFrameTime = keyFrameTimes.get(keyFrameTimes.size() - 1);

      TDoubleArrayList normalizedKeyFrameTimes = new TDoubleArrayList();
      trajectoryPointOptimizer.getWaypointTimes(normalizedKeyFrameTimes);

      keyFrameTimes.clear();
      keyFrameTimes.add(0.0);
      for (int i = 0; i < normalizedKeyFrameTimes.size(); i++)
         keyFrameTimes.add(normalizedKeyFrameTimes.get(i) * lastKeyFrameTime);

      keyFrameTimes.add(lastKeyFrameTime);
   }

   public void computeVelocityBounds(double searchingTimeTick)
   {
      for (String jointName : jointNames)
         jointNameToTrajectoryHolderMap.get(jointName).computeVelocityBounds(searchingTimeTick);
      if (SAVE_TRAJECTORY_DATA)
         saveJointPositionAndVelocity(searchingTimeTick);
   }

   public double getJointVelocityUpperBound(String jointName)
   {
      return jointNameToTrajectoryHolderMap.get(jointName).getUpperBound();
   }

   public double getJointVelocityLowerBound(String jointName)
   {
      return jointNameToTrajectoryHolderMap.get(jointName).getLowerBound();
   }

   public void packOptimizedVelocities(KinematicsPlanningToolboxOutputStatus solution)
   {
      List<KinematicsToolboxOutputStatus> robotConfigurations = solution.getRobotConfigurations();
      for (int i = 0; i < robotConfigurations.size(); i++)
      {
         KinematicsToolboxOutputStatus keyFrameSolution = robotConfigurations.get(i);
         for (String jointName : jointNames)
         {
            float wayPointVelocity = (float) jointNameToTrajectoryHolderMap.get(jointName).getWaypointVelocity(i);
            keyFrameSolution.getDesiredJointVelocities().add(wayPointVelocity);
         }
      }
      solution.getKeyFrameTimes().clear();
      for (int i = 0; i < robotConfigurations.size(); i++)
      {
         solution.getKeyFrameTimes().add(keyFrameTimes.get(i));
      }
   }

   private int findTrajectoryIndex(double time)
   {
      int indexOfTrajectory = 0;
      for (int j = keyFrameTimes.size() - 1; j > 0; j--)
      {
         if (keyFrameTimes.get(j) < time)
         {
            indexOfTrajectory = j;
            break;
         }
      }
      return indexOfTrajectory;
   }

   private void saveJointPositionAndVelocity(double searchingTimeTick)
   {
      int numberOfTicks = (int) (keyFrameTimes.get(keyFrameTimes.size() - 1) / searchingTimeTick);
      FileWriter positionFW, velocityFW;
      try
      {
         positionFW = new FileWriter(new File(this.getClass().getSimpleName() + "_position.csv"));
         velocityFW = new FileWriter(new File(this.getClass().getSimpleName() + "_velocity.csv"));

         positionFW.write(String.format("time"));
         velocityFW.write(String.format("time"));
         for (String jointName : jointNames)
         {
            positionFW.write(String.format("\t%s", jointName));
            velocityFW.write(String.format("\t%s", jointName));
         }
         positionFW.write(System.lineSeparator());
         velocityFW.write(System.lineSeparator());

         double time = 0.0;
         for (int i = 0; i < numberOfTicks; i++)
         {
            int indexOfTrajectory = findTrajectoryIndex(time);

            positionFW.write(String.format("%.4f (%d)", time, indexOfTrajectory));
            velocityFW.write(String.format("%.4f (%d)", time, indexOfTrajectory));
            for (String jointName : jointNames)
            {
               Trajectory trajectory = jointNameToTrajectoryHolderMap.get(jointName).getTrajectory(indexOfTrajectory);
               trajectory.compute(time);

               positionFW.write(String.format("\t%.4f", trajectory.getPosition()));
               velocityFW.write(String.format("\t%.4f", trajectory.getVelocity()));
            }

            positionFW.write(System.lineSeparator());
            velocityFW.write(System.lineSeparator());
            time += searchingTimeTick;
         }

         positionFW.close();
         velocityFW.close();
      }
      catch (IOException ex)
      {
         ex.printStackTrace();
      }
   }

   private class OneDoFJointTrajectoryHolder
   {
      private final TDoubleArrayList positions = new TDoubleArrayList();
      private final TDoubleArrayList velocities = new TDoubleArrayList();

      private final List<Trajectory> trajectories = new ArrayList<Trajectory>();

      private double velocityUpperBounds;
      private double velocityLowerBounds;

      public void clear()
      {
         positions.clear();
         velocities.clear();
      }

      public void addJointVelocity(double jointVelocity)
      {
         velocities.add(jointVelocity);
      }

      public void appendTrajectoryPoint(double jointPosition)
      {
         positions.add(jointPosition);
      }

      private void computeTrajectory()
      {
         trajectories.clear();
         for (int i = 0; i < positions.size() - 1; i++)
         {
            Trajectory cubic = new Trajectory(4);
            cubic.setCubic(keyFrameTimes.get(i), keyFrameTimes.get(i + 1), positions.get(i), velocities.get(i), positions.get(i + 1), velocities.get(i + 1));
            trajectories.add(cubic);
         }
      }

      public void computeVelocityBounds(double searchingTimeTick)
      {
         computeTrajectory();
         int numberOfTicks = (int) (keyFrameTimes.get(keyFrameTimes.size() - 1) / searchingTimeTick);
         double time = 0.0;
         TDoubleArrayList velocities = new TDoubleArrayList();
         for (int i = 0; i < numberOfTicks; i++)
         {
            int indexOfTrajectory = findTrajectoryIndex(time);
            Trajectory trajectory = trajectories.get(indexOfTrajectory);
            trajectory.compute(time);
            velocities.add(trajectory.getVelocity());

            time += searchingTimeTick;
         }
         velocityLowerBounds = velocities.min();
         velocityUpperBounds = velocities.max();
      }

      public double getFirstPoint()
      {
         return positions.get(0);
      }

      public double getLastPoint()
      {
         return positions.get(positions.size() - 1);
      }

      public double getWayPoint(int i)
      {
         return positions.get(i + 1);
      }

      public double getWaypointVelocity(int i)
      {
         return velocities.get(i);
      }

      public Trajectory getTrajectory(int i)
      {
         return trajectories.get(i);
      }

      public double getUpperBound()
      {
         return velocityUpperBounds;
      }

      public double getLowerBound()
      {
         return velocityLowerBounds;
      }
   }
}
