package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.KinematicsToolboxRigidBodyMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxSettings;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialData;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialNode;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class ConstrainedRigidBodyTrajectory
{
   private static final boolean VERBOSE = false;

   private final RigidBody rigidBody;

   private final TDoubleArrayList waypointTimes = new TDoubleArrayList();
   private final ArrayList<Pose3D> waypoints = new ArrayList<Pose3D>();

   private final SelectionMatrix6D trajectorySelectionMatrix = new SelectionMatrix6D();
   private final SelectionMatrix6D explorationSelectionMatrix = new SelectionMatrix6D();

   private final List<ConfigurationSpaceName> explorationConfigurationSpaces = new ArrayList<>();

   //private final TDoubleArrayList explorationRangeAmplitudes = new TDoubleArrayList();

   private final TDoubleArrayList explorationRangeUpperLimits = new TDoubleArrayList();
   private final TDoubleArrayList explorationRangeLowerLimits = new TDoubleArrayList();

   private final Pose3D controlFramePose = new Pose3D();

   private final RigidBodyTransform initialGuessResult = new RigidBodyTransform();
   private final RigidBodyTransform initialGuessHolder = new RigidBodyTransform();
   private final RigidBodyTransform initialGuessCandidate = new RigidBodyTransform();

   private final boolean hasTrajectoryCommand;

   private double weight;
   private static double DEFAULT_WEIGHT = 20.0;

   // For given trajectory and appending exploration transform.
   public ConstrainedRigidBodyTrajectory(RigidBody rigidBody, WaypointBasedTrajectoryCommand trajectoryCommand,
                                         RigidBodyExplorationConfigurationCommand explorationCommand)
   {
      this.rigidBody = rigidBody;
      if (trajectoryCommand != null)
      {
         for (int i = 0; i < trajectoryCommand.getNumberOfWaypoints(); i++)
         {
            waypointTimes.add(trajectoryCommand.getWaypointTime(i));
            waypoints.add(new Pose3D(trajectoryCommand.getWaypoint(i)));
         }
         trajectorySelectionMatrix.set(trajectoryCommand.getSelectionMatrix());

         FramePose3D controlFramePose = new FramePose3D(trajectoryCommand.getControlFramePose());
         controlFramePose.changeFrame(trajectoryCommand.getEndEffector().getBodyFixedFrame());
         this.controlFramePose.set(controlFramePose);
         this.hasTrajectoryCommand = true;
         if (Double.isNaN(trajectoryCommand.getWeight()))
            weight = DEFAULT_WEIGHT;
         else
            weight = trajectoryCommand.getWeight();
      }
      else
      {
         waypointTimes.add(0.0);
         waypointTimes.add(Double.MAX_VALUE);
         Pose3D originOfRigidBody = new Pose3D(rigidBody.getBodyFixedFrame().getTransformToWorldFrame());
         waypoints.add(originOfRigidBody);
         waypoints.add(originOfRigidBody);
         trajectorySelectionMatrix.clearSelection();

         controlFramePose.setToZero();
         this.hasTrajectoryCommand = false;
         weight = DEFAULT_WEIGHT;
      }

      explorationSelectionMatrix.clearSelection();
      explorationConfigurationSpaces.clear();

      explorationRangeUpperLimits.reset();
      explorationRangeLowerLimits.reset();

      if (explorationCommand != null && explorationCommand.getNumberOfDegreesOfFreedomToExplore() > 0)
      {
         for (int i = 0; i < explorationCommand.getNumberOfDegreesOfFreedomToExplore(); i++)
         {
            WholeBodyTrajectoryToolboxHelper.setSelectionMatrix(explorationSelectionMatrix, explorationCommand.getDegreeOfFreedomToExplore(i), true);

            explorationRangeUpperLimits.add(explorationCommand.getExplorationRangeUpperLimits(i));
            explorationRangeLowerLimits.add(explorationCommand.getExplorationRangeLowerLimits(i));
            explorationConfigurationSpaces.add(explorationCommand.getDegreeOfFreedomToExplore(i));
         }
      }
   }

   public boolean hasTrajectoryCommand()
   {
      return hasTrajectoryCommand;
   }

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }

   public SelectionMatrix6D getSelectionMatrix()
   {
      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.clearSelection();

      if (trajectorySelectionMatrix.getLinearPart().isXSelected() || explorationSelectionMatrix.getLinearPart().isXSelected())
         selectionMatrix.getLinearPart().selectXAxis(true);

      if (trajectorySelectionMatrix.getLinearPart().isYSelected() || explorationSelectionMatrix.getLinearPart().isYSelected())
         selectionMatrix.getLinearPart().selectYAxis(true);

      if (trajectorySelectionMatrix.getLinearPart().isZSelected() || explorationSelectionMatrix.getLinearPart().isZSelected())
         selectionMatrix.getLinearPart().selectZAxis(true);

      if (trajectorySelectionMatrix.getAngularPart().isXSelected() || explorationSelectionMatrix.getAngularPart().isXSelected())
         selectionMatrix.getAngularPart().selectXAxis(true);

      if (trajectorySelectionMatrix.getAngularPart().isYSelected() || explorationSelectionMatrix.getAngularPart().isYSelected())
         selectionMatrix.getAngularPart().selectYAxis(true);

      if (trajectorySelectionMatrix.getAngularPart().isZSelected() || explorationSelectionMatrix.getAngularPart().isZSelected())
         selectionMatrix.getAngularPart().selectZAxis(true);

      return selectionMatrix;
   }

   public Pose3D getPose(double time)
   {
      Pose3D current = new Pose3D();

      Pose3D previous = null;
      Pose3D next = null;
      double t0 = Double.NaN;
      double tf = Double.NaN;

      for (int i = 1; i < waypoints.size(); i++)
      {
         t0 = waypointTimes.get(i - 1);
         tf = waypointTimes.get(i);
         previous = waypoints.get(i - 1);
         next = waypoints.get(i);
         if (time < tf)
         {
            break;
         }
      }

      double alpha = (time - t0) / (tf - t0);
      alpha = MathTools.clamp(alpha, 0, 1);

      current.interpolate(previous, next, alpha);

      return current;
   }

   private Pose3D appendPoseToTrajectory(double timeInTrajectory, Pose3D poseToAppend)
   {
      Pose3D pose = getPose(timeInTrajectory);

      if (hasTrajectoryCommand)
         pose.appendTransform(new RigidBodyTransform(initialGuessResult));

      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform(poseToAppend.getOrientation(), poseToAppend.getPosition());
      pose.appendTransform(rigidBodyTransform);

      return pose;
   }

   public KinematicsToolboxRigidBodyMessage createMessage(double timeInTrajectory, Pose3D poseToAppend)
   {
      Pose3D desiredEndEffectorPose = appendPoseToTrajectory(timeInTrajectory, poseToAppend);

      KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(rigidBody);
      message.setDesiredPose(desiredEndEffectorPose);
      message.setControlFramePose(controlFramePose);
      message.setSelectionMatrix(getSelectionMatrix());
      message.setWeight(weight); // Sylvain's value :: 0.5

      return message;
   }

   public Pose3D getPoseToWorldFrame(Pose3D poseToAppend)
   {
      return appendPoseToTrajectory(0.0, poseToAppend);
   }

   public Pose3D getPoseToWorldFrame(SpatialNode spatialNode)
   {
      return appendPoseToTrajectory(spatialNode.getTime(), spatialNode.getSpatialData(getRigidBody()));
   }

   public void appendRandomSpatial(SpatialData spatialData)
   {
      // TODO: need a boolean variable to enable or disable exploration.
      // TODO: that is for dual hand reaching.
      RigidBodyTransform randomPose = new RigidBodyTransform();

      String[] configurationNames = new String[explorationConfigurationSpaces.size()];
      double[] configurationData = new double[explorationConfigurationSpaces.size()];

      for (int i = 0; i < explorationConfigurationSpaces.size(); i++)
      {
         ConfigurationSpaceName configurationSpaceName = explorationConfigurationSpaces.get(i);

         double lowerBound = 1.0 * explorationRangeLowerLimits.get(i);
         double upperBound = 1.0 * explorationRangeUpperLimits.get(i);
         double value = RandomNumbers.nextDouble(WholeBodyTrajectoryToolboxSettings.randomManager, lowerBound, upperBound);

         configurationNames[i] = rigidBody + "_" + configurationSpaceName.name();
         configurationData[i] = value;

         if (VERBOSE)
            PrintTools.info("" + i + " " + rigidBody + " " + value + " " + lowerBound + " " + upperBound);

         switch (configurationSpaceName)
         {
         case X:
            randomPose.appendTranslation(value, 0.0, 0.0);
            break;
         case Y:
            randomPose.appendTranslation(0.0, value, 0.0);
            break;
         case Z:
            randomPose.appendTranslation(0.0, 0.0, value);
            break;
         case ROLL:
            randomPose.appendRollRotation(value);
            break;
         case PITCH:
            randomPose.appendPitchRotation(value);
            break;
         case YAW:
            randomPose.appendYawRotation(value);
            break;
         default:
            break;
         }
      }
      initialGuessCandidate.set(randomPose);

      spatialData.appendSpatial(getRigidBody().getName(), configurationNames, configurationData, randomPose);
   }

   public void holdConfiguration(RigidBody rigidBody)
   {
      if (hasTrajectoryCommand)
      {
         initialGuessHolder.set(initialGuessCandidate);
      }
      else
      {
         initialGuessHolder.set(rigidBody.getBodyFixedFrame().getTransformToWorldFrame());
      }
   }

   public void updateInitialResult()
   {
      if (hasTrajectoryCommand)
      {
         initialGuessResult.set(initialGuessHolder);
      }
      else
      {
         waypoints.set(0, new Pose3D(initialGuessHolder));
         waypoints.set(1, new Pose3D(initialGuessHolder));
      }
   }
}
