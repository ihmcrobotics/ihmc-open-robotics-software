package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.fest.swing.util.Pair;

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.robotics.math.trajectories.generators.TrajectoryPointOptimizer;

public class KeyFrameBasedTrajectoryGenerator
{
   /*
    * keyFrames includes initial configuration. keyFrameTimes includes 0.0 (s)
    * for initial configuration.
    */
   private final List<KinematicsToolboxOutputStatus> keyFrames = new ArrayList<KinematicsToolboxOutputStatus>();
   private final TDoubleArrayList keyFrameTimes = new TDoubleArrayList();

   private final TrajectoryPointOptimizer trajectoryPointOptimizer;

   private final List<String> jointNames = new ArrayList<String>();
   private final Map<String, List<Trajectory>> jointNameToTrajectoriesMap = new HashMap<>();
   private final Map<String, Pair<Double, Double>> jointNameToVelocityBoundMap = new HashMap<>();
   private final Map<String, TDoubleArrayList> jointNameToPositionsMap = new HashMap<>();
   private final Map<String, TDoubleArrayList> jointNameToVelocitiesMap = new HashMap<>();

   private final KinematicsToolboxOutputConverter converter;

   private final boolean SAVE_TRAJECTORY_DATA = false;

   public KeyFrameBasedTrajectoryGenerator(DRCRobotModel drcRobotModel)
   {
      OneDoFJointBasics[] allJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(drcRobotModel.createFullRobotModel());
      int numberOfWholeBodyJoints = allJointsExcludingHands.length;
      trajectoryPointOptimizer = new TrajectoryPointOptimizer(numberOfWholeBodyJoints);
      converter = new KinematicsToolboxOutputConverter(drcRobotModel);

      for (int i = 0; i < numberOfWholeBodyJoints; i++)
      {
         jointNames.add(allJointsExcludingHands[i].getName());
         LogTools.info(""+i+" "+allJointsExcludingHands[i].getName());
      }

      for (String jointName : jointNames)
         jointNameToTrajectoriesMap.put(jointName, new ArrayList<Trajectory>());
   }

   public void computeOptimizingKeyFrameTimes()
   {
      // TODO : Don't forget! multiply last key frame time to recover from normalized time.
   }

   public void compute()
   {
      // normalize key frame times.
      double lastKeyFrameTime = keyFrameTimes.get(keyFrameTimes.size() - 1);
      TDoubleArrayList normalizedKeyFrameTimes = new TDoubleArrayList();
      for (int i = 1; i < keyFrameTimes.size() - 1; i++)
         normalizedKeyFrameTimes.add(keyFrameTimes.get(i) / lastKeyFrameTime);

      // compute.
      trajectoryPointOptimizer.computeForFixedTime(normalizedKeyFrameTimes);

      // save optimized joint velocities on map.
      for (String jointName : jointNames)
         jointNameToVelocitiesMap.get(jointName).add(0.0);

      TDoubleArrayList velocityToPack = new TDoubleArrayList();
      for (int i = 1; i < keyFrames.size() - 1; i++)
      {
         trajectoryPointOptimizer.getWaypointVelocity(velocityToPack, i - 1);
         for (int j = 0; j < jointNames.size(); j++)
         {
            jointNameToVelocitiesMap.get(jointNames.get(j)).add(velocityToPack.get(j) / lastKeyFrameTime);
         }
      }
      for (String jointName : jointNames)
         jointNameToVelocitiesMap.get(jointName).add(0.0);
   }

   public void addInitialConfiguration(KinematicsToolboxOutputStatus initialConfiguration)
   {
      if (keyFrames.size() > 0 || keyFrameTimes.size() > 0)
      {
         LogTools.warn("keyFrames should be cleared");
         clear();
      }
      keyFrames.add(initialConfiguration);
      keyFrameTimes.add(0.0);
   }

   public void addKeyFrames(List<KinematicsToolboxOutputStatus> solutionKeyFrames, TDoubleArrayList solutionKeyFrameTimes)
   {
      keyFrames.addAll(solutionKeyFrames);
      keyFrameTimes.addAll(solutionKeyFrameTimes);
   }

   private void clear()
   {
      keyFrames.clear();
      keyFrameTimes.clear();
   }

   public void initializeTrajectoryGenerator()
   {
      ArrayList<TDoubleArrayList> wayPointSets = new ArrayList<>();
      for (int i = 1; i < keyFrames.size() - 1; i++)
      {
         TDoubleArrayList wayPointsForAKeyFrameTime = new TDoubleArrayList();
         for (String jointName : jointNames)
         {
            KinematicsToolboxOutputStatus keyFrame = keyFrames.get(i);
            converter.updateFullRobotModel(keyFrame);
            double wayPoint = converter.getFullRobotModel().getOneDoFJointByName(jointName).getQ();
            wayPointsForAKeyFrameTime.add(wayPoint);
         }
         wayPointSets.add(wayPointsForAKeyFrameTime);
      }

      TDoubleArrayList firstWayPoints = new TDoubleArrayList();
      TDoubleArrayList lastWayPoints = new TDoubleArrayList();
      TDoubleArrayList firstWayPointVelocities = new TDoubleArrayList();
      TDoubleArrayList lastWayPointVelocities = new TDoubleArrayList();
      for (String jointName : jointNames)
      {
         KinematicsToolboxOutputStatus firstKeyFrame = keyFrames.get(0);
         converter.updateFullRobotModel(firstKeyFrame);
         double firstWayPoint = converter.getFullRobotModel().getOneDoFJointByName(jointName).getQ();
         firstWayPoints.add(firstWayPoint);

         KinematicsToolboxOutputStatus lastKeyFrame = keyFrames.get(keyFrames.size() - 1);
         converter.updateFullRobotModel(lastKeyFrame);
         double lastWayPoint = converter.getFullRobotModel().getOneDoFJointByName(jointName).getQ();
         lastWayPoints.add(lastWayPoint);

         firstWayPointVelocities.add(0.0);
         lastWayPointVelocities.add(0.0);
      }

      trajectoryPointOptimizer.setEndPoints(firstWayPoints, firstWayPointVelocities, lastWayPoints, lastWayPointVelocities);
      trajectoryPointOptimizer.setWaypoints(wayPointSets);

      for (int i = 0; i < jointNames.size(); i++)
      {
         TDoubleArrayList jointPositions = new TDoubleArrayList();
         jointPositions.add(firstWayPoints.get(i));
         for (int j = 0; j < wayPointSets.size(); j++)
            jointPositions.add(wayPointSets.get(j).get(i));
         jointPositions.add(lastWayPoints.get(i));

         jointNameToPositionsMap.put(jointNames.get(i), jointPositions);
         jointNameToVelocitiesMap.put(jointNames.get(i), new TDoubleArrayList());
      }
   }

   public void computeVelocityBound(double searchingTimeTick)
   {
      computeOptimizedTrajectories();
      int numberOfTicks = (int) (keyFrameTimes.get(keyFrameTimes.size() - 1) / searchingTimeTick);
      for (String jointName : jointNames)
      {
         double time = 0.0;
         TDoubleArrayList velocities = new TDoubleArrayList();
         for (int i = 0; i < numberOfTicks; i++)
         {
            int indexOfTrajectory = findTrajectoryIndex(time);
            Trajectory trajectory = jointNameToTrajectoriesMap.get(jointName).get(indexOfTrajectory);
            trajectory.compute(time);
            velocities.add(trajectory.getVelocity());

            time += searchingTimeTick;
         }
         Pair<Double, Double> minMaxVelocity = new Pair<Double, Double>(velocities.min(), velocities.max());
         jointNameToVelocityBoundMap.put(jointName, minMaxVelocity);
      }
      if (SAVE_TRAJECTORY_DATA)
         saveJointPositionAndVelocity(searchingTimeTick);
   }

   private void computeOptimizedTrajectories()
   {
      for (String jointName : jointNames)
      {
         TDoubleArrayList positions = jointNameToPositionsMap.get(jointName);
         TDoubleArrayList velocities = jointNameToVelocitiesMap.get(jointName);
         List<Trajectory> trajectories = new ArrayList<Trajectory>();
         for (int i = 0; i < keyFrameTimes.size() - 1; i++)
         {
            Trajectory cubic = new Trajectory(4);
            cubic.setCubic(keyFrameTimes.get(i), keyFrameTimes.get(i + 1), positions.get(i), velocities.get(i), positions.get(i + 1), velocities.get(i + 1));
            trajectories.add(cubic);
         }
         jointNameToTrajectoriesMap.put(jointName, trajectories);
      }
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
               Trajectory trajectory = jointNameToTrajectoriesMap.get(jointName).get(indexOfTrajectory);
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

      LogTools.info("done");
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

   public double getJointVelocityUpperBound(String jointName)
   {
      return jointNameToVelocityBoundMap.get(jointName).ii;
   }

   public double getJointVelocityLowerBound(String jointName)
   {
      return jointNameToVelocityBoundMap.get(jointName).i;
   }
}