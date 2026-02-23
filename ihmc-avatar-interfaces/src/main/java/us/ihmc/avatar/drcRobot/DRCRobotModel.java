package us.ihmc.avatar.drcRobot;

import us.ihmc.avatar.AvatarSimulatedHandControlThread;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.multiContact.pushRecovery.ReactiveBracingPlanner;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParametersBasics;
import us.ihmc.footstepPlanning.FastLocomotionParameters;
import us.ihmc.footstepPlanning.LocomotionParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.handsros2.HandModel;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.scs2.simulation.collision.CollidableHelper;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.SimulatedFullHumanoidRobotModelFactory;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

import java.nio.file.Path;

public interface DRCRobotModel extends SimulatedFullHumanoidRobotModelFactory, WholeBodyControllerParameters<RobotSide>
{
   public default RobotTarget getTarget()
   {
      return null;
   }

   public abstract HumanoidJointNameMap getJointMap();

   public abstract RobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup();

   default RobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return getDefaultRobotInitialSetup(groundHeight, initialYaw, 0, 0);
   }

   default RobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw, double x, double y)
   {
      RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = getDefaultRobotInitialSetup();
      robotInitialSetup.setInitialGroundHeight(groundHeight);
      robotInitialSetup.setInitialYaw(initialYaw);
      robotInitialSetup.setOffset(new Vector3D(x, y, 0.0));
      return robotInitialSetup;
   }
   
   default RobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw, double x, double y, double z)
   {
      RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = getDefaultRobotInitialSetup();
      robotInitialSetup.setInitialGroundHeight(groundHeight);
      robotInitialSetup.setInitialYaw(initialYaw);
      robotInitialSetup.setOffset(new Vector3D(x, y, z));
      return robotInitialSetup;
   }

   public default double[] getPresetArmConfiguration(RobotSide side, PresetArmConfiguration presetArmConfiguration)
   {
      throw new RuntimeException("Not implemented");
   }

   public abstract HandModel getHandModel(RobotSide side);

   public default SideDependentList<HandModel> getHandModels()
   {
      SideDependentList<HandModel> handModels = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         HandModel handModel = getHandModel(side);
         if (handModel != null)
         {
            handModels.put(side, handModel);
         }
      }
      return handModels;
   }

   default double getSlowestControllerDT()
   {
      double dt = 0.0;
      for (HighLevelControllerName name : HighLevelControllerName.values)
      {
         double stateDT = getHighLevelControllerParameters().getControlDT(name);
         if (stateDT > 0.0)
            dt = Math.max(stateDT, dt);
      }
      return dt;
   }

   default double getFastestControllerDT()
   {
      double dt = Double.POSITIVE_INFINITY;
      for (HighLevelControllerName name : HighLevelControllerName.values)
      {
         double stateDT = getHighLevelControllerParameters().getControlDT(name);
         if (stateDT > 0.0)
            dt = Math.min(stateDT, dt);
      }
      return dt;
   }

   default double getSimulatedHandControlDT()
   {
      return getFastestControllerDT();
   }

   default double getFeedbackControllerDT()
   {
      return 0.0;
   }


   public abstract double getSimulateDT();

   public abstract double getEstimatorDT();

   default double getStepGeneratorDT()
   {
      return 10.0 * getFastestControllerDT();
   }

   public default AvatarSimulatedHandControlThread createSimulatedHandController(RealtimeROS2Node realtimeROS2Node, boolean kinematicsSimulation)
   {
      return null;
   }

   public abstract DataServerSettings getLogSettings();

   public abstract LogModelProvider getLogModelProvider();

   public abstract String getSimpleRobotName();

   public default LocomotionParameters getLocomotionParameters()
   {
      return null;
   }

   public default FastLocomotionParameters getFastLocomotionParameters()
   {
      return null;
   }

   public default DefaultFootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return null;
   }

   default DefaultFootstepPlannerParametersBasics getFootstepPlannerParameters(String fileNameSuffix)
   {
      return null;
   }

   default AStarBodyPathPlannerParametersBasics getAStarBodyPathPlannerParameters()
   {
      return null;
   }

   default String getStepReachabilityResourceName()
   {
      return null;
   }

   default StepReachabilityData getStepReachabilityData()
   {
      return null;
   }

   default SwingPlannerParametersBasics getSwingPlannerParameters(String fileNameSuffix)
   {
      return null;
   }

   default SwingPlannerParametersBasics getSwingPlannerParameters()
   {
      return null;
   }

   public HighLevelControllerParameters getHighLevelControllerParameters();

   default RobotCollisionModel getHumanoidRobotKinematicsCollisionModel()
   {
      return null;
   }

   /**
    * Gets the collision model for this robot to use with {@link us.ihmc.scs2.simulation.physicsEngine.impulseBased.ImpulseBasedPhysicsEngine}.

    * @param helper                    the helper to use when creating the {@link Collidable}s for
    *                                  generating the collidable masks and groups.
    * @param robotCollisionMask        the mask for the robot collidables that are supposed to interact
    *                                  with the environment.
    * @param environmentCollisionMasks the masks used for the environment collidables needed to create
    *                                  the collision group for the robot collidables that are to
    *                                  interact with the environment.
    * @return the robot collision model used to create the robot's {@link Collidable}s.
    */
   default RobotCollisionModel getSimulationRobotCollisionModel(CollidableHelper helper, String robotCollisionMask, String... environmentCollisionMasks)
   {
      return null;
   }

   default RobotLowLevelMessenger newRobotLowLevelMessenger(ROS2Node ros2Node)
   {
      return null;
   }

   default Path getMultiContactScriptPath()
   {
      return null;
   }

   default RobotVersion getRobotVersion()
   {
      return null;
   }

   public default RigidBodyTransform getHandGraphicToHandFrameTransform(RobotSide side)
   {
      return new RigidBodyTransform();
   }

   public default RigidBodyTransform getChestGraphicToFrameTransform()
   {
      return new RigidBodyTransform();
   }

   default ReactiveBracingPlanner getReactiveBracingPlanner()
   {
      return null;
   }
}
