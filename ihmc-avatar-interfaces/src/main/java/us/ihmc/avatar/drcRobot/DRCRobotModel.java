package us.ihmc.avatar.drcRobot;

import us.ihmc.avatar.SimulatedLowLevelOutputWriter;
import us.ihmc.avatar.drcRobot.shapeContactSettings.DRCRobotModelShapeCollisionSettings;
import us.ihmc.avatar.drcRobot.shapeContactSettings.DefaultShapeCollisionSettings;
import us.ihmc.avatar.factory.SimulatedHandControlTask;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.avatar.ros.WallTimeBasedROSClockCalculator;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SliderBoardParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationToolkit.physicsEngine.ExperimentalSimulation;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCOutputProcessor;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.wholeBodyController.SimulatedFullHumanoidRobotModelFactory;
import us.ihmc.wholeBodyController.UIParameters;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public interface DRCRobotModel extends SimulatedFullHumanoidRobotModelFactory, WholeBodyControllerParameters<RobotSide>
{
   public default RobotTarget getTarget()
   {
      return null;
   }

   public abstract HumanoidJointNameMap getJointMap();

   public abstract DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw);

   default DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw, double x, double y)
   {
      return getDefaultRobotInitialSetup(groundHeight, initialYaw);
   }

   public abstract HandModel getHandModel();

   public abstract double getSimulateDT();

   public abstract double getEstimatorDT();

   public default RobotROSClockCalculator getROSClockCalculator()
   {
      return new WallTimeBasedROSClockCalculator();
   }

   public default DRCSensorSuiteManager getSensorSuiteManager()
   {
      return getSensorSuiteManager(null);
   }

   public abstract DRCSensorSuiteManager getSensorSuiteManager(ROS2NodeInterface ros2Node);

   public default SimulatedHandControlTask createSimulatedHandController(FloatingRootJointRobot simulatedRobot, RealtimeROS2Node realtimeROS2Node)
   {
      return null;
   }

   public abstract DataServerSettings getLogSettings();

   public abstract LogModelProvider getLogModelProvider();

   public abstract String getSimpleRobotName();

   public abstract CollisionBoxProvider getCollisionBoxProvider();

   public default SliderBoardParameters getSliderBoardParameters()
   {
      return new SliderBoardParameters();
   }

   /**
    * Override this method to create a custom output processor to be used with this robot.
    * <p>
    * <b> This output writer is meant to be used in simulation only.
    * </p>
    * 
    * @param humanoidFloatingRootJointRobot Optional handle to the robot to allow directly writing to
    *                                       the joints.
    * @return the custom output processor.
    */
   public default DRCOutputProcessor getCustomSimulationOutputProcessor(HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot)
   {
      return null;
   }

   /**
    * Override this method to create a custom output writer to be used with this robot.
    * <p>
    * <b> This output writer is meant to be used in simulation only.
    * </p>
    * 
    * @param JointDesiredOutputWriter The outputWriter to use. If null is returned, no output writer is
    *                                 used.
    * @return the custom output writer.
    */
   public default JointDesiredOutputWriter getCustomSimulationOutputWriter(HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot,
                                                                           HumanoidRobotContextData contextData)
   {
      return new SimulatedLowLevelOutputWriter(humanoidFloatingRootJointRobot, true);
   }

   /**
    * Returns a factory for creating low-level joint controller that can be used to simulate for
    * instance a joint position controller.
    * 
    * @return the low-level controller factory to use in simulation.
    */
   public default SimulationLowLevelControllerFactory getSimulationLowLevelControllerFactory()
   {
      return new DefaultSimulationLowLevelControllerFactory(getJointMap(), getSimulateDT());
   }

   /**
    * @return parameters used in the user interface only.
    */
   public default UIParameters getUIParameters()
   {
      return null;
   }

   public default FootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return null;
   }

   default FootstepPlannerParametersBasics getFootstepPlannerParameters(String fileNameSuffix)
   {
      return null;
   }

   default VisibilityGraphsParametersBasics getVisibilityGraphsParameters()
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

   public default DRCRobotModelShapeCollisionSettings getShapeCollisionSettings()
   {
      return new DefaultShapeCollisionSettings();
   }

   default RobotCollisionModel getHumanoidRobotKinematicsCollisionModel()
   {
      return null;
   }

   /**
    * Gets the collision model for this robot to use with {@link ExperimentalSimulation}.
    * <p>
    * {@link ExperimentalSimulation} can be used instead of the default SCS physics engine using
    * {@link DRCSCSInitialSetup#setUseExperimentalPhysicsEngine(boolean)}.
    * </p>
    * 
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
}
