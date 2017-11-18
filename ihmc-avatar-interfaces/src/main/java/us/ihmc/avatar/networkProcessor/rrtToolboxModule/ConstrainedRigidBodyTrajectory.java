package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNode;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class ConstrainedRigidBodyTrajectory
{
   private final RigidBody rigidBody;

   private final TDoubleArrayList waypointTimes = new TDoubleArrayList();
   private final ArrayList<Pose3D> waypoints = new ArrayList<Pose3D>();

   /**
    * The first way point time.
    */
   private final double t0;

   /**
    * The final way point time.
    */
   private final double tf;

   private final SelectionMatrix6D trajectorySelectionMatrix;
   private final SelectionMatrix6D explorationSelectionMatrix;

   private final List<ConfigurationSpaceName> explorationConfigurationSpaces = new ArrayList<>();
   private final TDoubleArrayList explorationRangeUpperLimits = new TDoubleArrayList();
   private final TDoubleArrayList explorationRangeLowerLimits = new TDoubleArrayList();

   private final Point3D controlFramePoint;
   private final Quaternion controlFrameOrienataion;

   // For given trajectory and appending exploration transform.
   public ConstrainedRigidBodyTrajectory(WaypointBasedTrajectoryCommand trajectoryCommand, RigidBodyExplorationConfigurationCommand explorationCommand)
   {
      rigidBody = explorationCommand.getRigidBody();

      waypointTimes.clear();
      waypoints.clear();
      for (int i = 0; i < trajectoryCommand.getNumberOfWaypoints(); i++)
      {
         waypointTimes.add(trajectoryCommand.getWaypointTime(i));
         waypoints.add(new Pose3D(trajectoryCommand.getWaypoint(i)));
      }

      t0 = trajectoryCommand.getWaypointTime(0);
      tf = trajectoryCommand.getLastWaypointTime();

      trajectorySelectionMatrix = new SelectionMatrix6D();
      trajectorySelectionMatrix.resetSelection();
      explorationSelectionMatrix = new SelectionMatrix6D();
      explorationSelectionMatrix.clearSelection();

      if (trajectoryCommand.getNumberOfUnconstrainedDegreesOfFreedom() > 0)
      {
         for (int i = 0; i < trajectoryCommand.getNumberOfUnconstrainedDegreesOfFreedom(); i++)
         {
            WholeBodyTrajectoryToolboxHelper.setSelectionMatrix(trajectorySelectionMatrix, trajectoryCommand.getUnconstrainedDegreeOfFreedom(i), false);
         }
      }

      if (trajectoryCommand.getControlFramePositionEndEffector() != null)
         controlFramePoint = new Point3D(trajectoryCommand.getControlFramePositionEndEffector());
      else
         controlFramePoint = new Point3D();

      if (trajectoryCommand.getControlFrameOrientationEndEffector() != null)
         controlFrameOrienataion = new Quaternion(trajectoryCommand.getControlFrameOrientationEndEffector());
      else
         controlFrameOrienataion = new Quaternion();

      setRigidBodyExplorationConfigurationCommand(explorationCommand);
   }

   // Exploration only.
   public ConstrainedRigidBodyTrajectory(FullHumanoidRobotModel fullRobotModel, RigidBodyExplorationConfigurationCommand explorationCommand)
   {
      rigidBody = explorationCommand.getRigidBody();

      waypointTimes.clear();
      waypoints.clear();
      waypointTimes.add(0.0);
      waypointTimes.add(Double.MAX_VALUE);
      Pose3D originOfRigidBody;
      if (rigidBody == fullRobotModel.getHand(RobotSide.LEFT))
         originOfRigidBody = new Pose3D(fullRobotModel.getHand(RobotSide.LEFT).getBodyFixedFrame().getTransformToWorldFrame());
      else if (rigidBody == fullRobotModel.getHand(RobotSide.RIGHT))
         originOfRigidBody = new Pose3D(fullRobotModel.getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToWorldFrame());
      else if (rigidBody == fullRobotModel.getChest())
         originOfRigidBody = new Pose3D(fullRobotModel.getChest().getBodyFixedFrame().getTransformToWorldFrame());
      else if (rigidBody == fullRobotModel.getPelvis())
         originOfRigidBody = new Pose3D(fullRobotModel.getPelvis().getBodyFixedFrame().getTransformToWorldFrame());
      else
         throw new RuntimeException("Unexpected rigid body: " + rigidBody.getName());
      waypoints.add(originOfRigidBody);
      waypoints.add(originOfRigidBody);

      controlFramePoint = new Point3D();
      controlFrameOrienataion = new Quaternion();

      t0 = 0.0;
      tf = Double.MAX_VALUE;

      trajectorySelectionMatrix = new SelectionMatrix6D();
      trajectorySelectionMatrix.clearSelection();
      explorationSelectionMatrix = new SelectionMatrix6D();
      explorationSelectionMatrix.clearSelection();

      setRigidBodyExplorationConfigurationCommand(explorationCommand);
   }

   private void setRigidBodyExplorationConfigurationCommand(RigidBodyExplorationConfigurationCommand explorationCommand)
   {
      explorationConfigurationSpaces.clear();
      explorationRangeUpperLimits.reset();
      explorationRangeLowerLimits.reset();

      if (explorationCommand.getNumberOfDegreesOfFreedomToExplore() > 0)
      {
         for (int i = 0; i < explorationCommand.getNumberOfDegreesOfFreedomToExplore(); i++)
         {
            WholeBodyTrajectoryToolboxHelper.setSelectionMatrix(explorationSelectionMatrix, explorationCommand.getDegreeOfFreedomToExplore(i), true);
            explorationRangeUpperLimits.add(explorationCommand.getExplorationUpperLimit(i));
            explorationRangeLowerLimits.add(explorationCommand.getExplorationLowerLimit(i));
            explorationConfigurationSpaces.add(explorationCommand.getDegreeOfFreedomToExplore(i));
         }
      }
   }

   public ArrayList<String> getExplorationConfigurationNames()
   {
      ArrayList<String> ret = new ArrayList<String>();
      for (int i = 0; i < getExplorationDimension(); i++)
         ret.add(getExplorationConfigurationName(i));
      return ret;
   }

   public TDoubleArrayList getExplorationRangeUpperLimits()
   {
      return explorationRangeUpperLimits;
   }

   public TDoubleArrayList getExplorationRangeLowerLimits()
   {
      return explorationRangeLowerLimits;
   }

   private String getExplorationConfigurationName(int i)
   {
      if (i >= explorationConfigurationSpaces.size())
         throw new RuntimeException("Try to get over size " + this);

      return getRigidBody().getName() + "_" + explorationConfigurationSpaces.get(i).name();
   }

   public int getExplorationDimension()
   {
      return explorationConfigurationSpaces.size();
   }

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }

   // this is for the kinematicsSolver.
   public SelectionMatrix6D getSelectionMatrix()
   {
      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

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

   public Pose3D getPoseFromTrajectory(CTTaskNode node, Map<Integer, RigidBody> map)
   {
      double time = node.getTime();
      MathTools.clamp(time, t0, tf);

      int dimensionOfExploration = getExplorationDimension();

      double[] configurations = new double[dimensionOfExploration];
      int curConfiguration = 0;
      for (int i = 0; i < map.size(); i++)
      {
         if (getRigidBody() == map.get(i + 1))
         {
            PrintTools.info(""+rigidBody +" "+node.getNodeData(i + 1));
            configurations[curConfiguration] = node.getNodeData(i + 1);
            curConfiguration++;
         }
      }

      Pose3D pose = getPose(time);
      
      for (int i = 0; i < configurations.length; i++)
      {
         appendConfiguration(pose, i, configurations[i]);
      }  
      
      return pose;
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

      // inverse control frame 
      RigidBodyTransform endEffectorToDesired = new RigidBodyTransform(controlFrameOrienataion, controlFramePoint);
      endEffectorToDesired.invert();
      
      current.appendTransform(endEffectorToDesired);
      
      return current;
   }

   private void appendConfiguration(Pose3D pose, int index, double configuration)
   {
      pose.appendTransform(explorationConfigurationSpaces.get(index).getLocalRigidBodyTransform(configuration));
   }


}
