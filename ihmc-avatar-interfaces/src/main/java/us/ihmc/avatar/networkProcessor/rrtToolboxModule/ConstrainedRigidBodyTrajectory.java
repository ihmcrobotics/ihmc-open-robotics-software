package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
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

   private final List<ConfigurationSpaceName> degreesOfFreedomToExplore = new ArrayList<>();
   private final TDoubleArrayList explorationRangeUpperLimits = new TDoubleArrayList();
   private final TDoubleArrayList explorationRangeLowerLimits = new TDoubleArrayList();

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

      if (trajectoryCommand.getNumberOfUnconstrainedDegreesOfFreedom() > 0)
      {
         for (int i = 0; i < trajectoryCommand.getNumberOfUnconstrainedDegreesOfFreedom(); i++)
         {
            WholeBodyTrajectoryToolboxHelper.setSelectionMatrix(trajectorySelectionMatrix, trajectoryCommand.getUnconstrainedDegreeOfFreedom(i), false);
         }
      }

      degreesOfFreedomToExplore.clear();
      explorationRangeUpperLimits.reset();
      explorationRangeLowerLimits.reset();
      explorationSelectionMatrix = new SelectionMatrix6D();
      explorationSelectionMatrix.clearSelection();

      if (explorationCommand.getNumberOfDegreesOfFreedomToExplore() > 0)
      {
         for (int i = 0; i < explorationCommand.getNumberOfDegreesOfFreedomToExplore(); i++)
         {
            WholeBodyTrajectoryToolboxHelper.setSelectionMatrix(explorationSelectionMatrix, explorationCommand.getDegreeOfFreedomToExplore(i), true);
            explorationRangeUpperLimits.add(explorationCommand.getExplorationUpperLimit(i));
            explorationRangeLowerLimits.add(explorationCommand.getExplorationLowerLimit(i));
            degreesOfFreedomToExplore.add(explorationCommand.getDegreeOfFreedomToExplore(i));
         }
      }
   }

   // Exploration only.
   public ConstrainedRigidBodyTrajectory(FullHumanoidRobotModel fullRobotModel, RigidBodyExplorationConfigurationCommand explorationCommand)
   {
      rigidBody = explorationCommand.getRigidBody();

      waypointTimes.clear();
      waypoints.clear();
      waypointTimes.add(0.0);
      Pose3D originOfRigidBody;      
      if(rigidBody == fullRobotModel.getHand(RobotSide.LEFT))
         originOfRigidBody = new Pose3D(fullRobotModel.getHand(RobotSide.LEFT).getBodyFixedFrame().getTransformToWorldFrame());
      
      else if(rigidBody == fullRobotModel.getHand(RobotSide.RIGHT))
         originOfRigidBody = new Pose3D(fullRobotModel.getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToWorldFrame());
      
      else if(rigidBody == fullRobotModel.getChest())
         originOfRigidBody = new Pose3D(fullRobotModel.getChest().getBodyFixedFrame().getTransformToWorldFrame());
      
      else if(rigidBody == fullRobotModel.getPelvis())
         originOfRigidBody = new Pose3D(fullRobotModel.getPelvis().getBodyFixedFrame().getTransformToWorldFrame());
      
      else      
         throw new RuntimeException("Unexpected rigid body: " + rigidBody.getName());
      
      waypoints.add(originOfRigidBody);

      t0 = 0.0;
      tf = Double.MAX_VALUE;

      trajectorySelectionMatrix = new SelectionMatrix6D();
      trajectorySelectionMatrix.clearSelection();

      degreesOfFreedomToExplore.clear();
      explorationRangeUpperLimits.reset();
      explorationRangeLowerLimits.reset();
      explorationSelectionMatrix = new SelectionMatrix6D();
      explorationSelectionMatrix.clearSelection();

      if (explorationCommand.getNumberOfDegreesOfFreedomToExplore() > 0)
      {
         for (int i = 0; i < explorationCommand.getNumberOfDegreesOfFreedomToExplore(); i++)
         {
            WholeBodyTrajectoryToolboxHelper.setSelectionMatrix(explorationSelectionMatrix, explorationCommand.getDegreeOfFreedomToExplore(i), true);
            explorationRangeUpperLimits.add(explorationCommand.getExplorationUpperLimit(i));
            explorationRangeLowerLimits.add(explorationCommand.getExplorationLowerLimit(i));
            degreesOfFreedomToExplore.add(explorationCommand.getDegreeOfFreedomToExplore(i));
         }
      }
   }

   public String getExplorationConfigurationName(int i)
   {
      if (i >= degreesOfFreedomToExplore.size())
         throw new RuntimeException("Try to get over size " + this);

      return getRigidBody().getName() + "_" + degreesOfFreedomToExplore.get(i).name();
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

   // TODO
   // from trajectoryFunction
   public Pose3D getPoseFromTrajectory(double time)
   {
      double putTime;

      if (time < t0)
         putTime = t0;
      else if (time > tf)
         putTime = tf;
      else
         ;

      return new Pose3D();
   }

   // TODO
   // default value
   public RigidBodyTransform getControlFrameTransformation()
   {
      return new RigidBodyTransform();
   }

   public double getRandomConfiguration(int i)
   {

      return 0.0;
   }

   // TODO
   // based on upper lower limit
   // some rigidbodydata has default region
   public RigidBodyTransform getRandomTransformation()
   {
      return new RigidBodyTransform();
   }
}
