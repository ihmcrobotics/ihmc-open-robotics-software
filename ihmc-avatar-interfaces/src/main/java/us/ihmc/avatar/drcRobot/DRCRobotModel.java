package us.ihmc.avatar.drcRobot;

import com.jme3.math.Transform;

import us.ihmc.avatar.SimulatedLowLevelOutputWriter;
import us.ihmc.avatar.drcRobot.shapeContactSettings.DRCRobotModelShapeCollisionSettings;
import us.ihmc.avatar.drcRobot.shapeContactSettings.DefaultShapeCollisionSettings;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SliderBoardParameters;
import us.ihmc.footstepPlanning.PlanarRegionFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.wholeBodyController.DRCOutputProcessor;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.SimulatedFullHumanoidRobotModelFactory;
import us.ihmc.wholeBodyController.UIParameters;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;

public interface DRCRobotModel extends SimulatedFullHumanoidRobotModelFactory, WholeBodyControllerParameters<RobotSide>
{
   public abstract DRCRobotJointMap getJointMap();

   public abstract DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw);

   public abstract HandModel getHandModel();

   public abstract Transform getJmeTransformWristToHand(RobotSide side);

   public abstract double getSimulateDT();

   public abstract double getEstimatorDT();

   public abstract double getStandPrepAngle(String jointName);

   public abstract DRCROSPPSTimestampOffsetProvider getPPSTimestampOffsetProvider();

   public abstract DRCSensorSuiteManager getSensorSuiteManager();

   public abstract MultiThreadedRobotControlElement createSimulatedHandController(FloatingRootJointRobot simulatedRobot,
                                                                                  ThreadDataSynchronizerInterface threadDataSynchronizer,
                                                                                  RealtimeRos2Node realtimeRos2Node,
                                                                                  CloseableAndDisposableRegistry closeableAndDisposableRegistry);

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
    * @param humanoidFloatingRootJointRobot Optional handle to the robot to allow directly writing
    *           to the joints.
    *
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
    * @param JointDesiredOutputWriter The outputWriter to use. If null is returned, no output writer
    *           is used.
    *
    * @return the custom output writer.
    */
   public default JointDesiredOutputWriter getCustomSimulationOutputWriter(HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot)
   {
      return new SimulatedLowLevelOutputWriter(humanoidFloatingRootJointRobot, true);
   }

   /**
    * @return parameters used in the user interface only.
    */
   public default UIParameters getUIParameters()
   {
      return null;
   }

   public default PlanarRegionFootstepPlanningParameters getPlanarRegionFootstepPlannerParameters()
   {
      return null;
   }

   public default FootstepPlannerParameters getFootstepPlannerParameters()
   {
      return null;
   }

   default VisibilityGraphsParameters getVisibilityGraphsParameters()
   {
      return null;
   }

   public HighLevelControllerParameters getHighLevelControllerParameters();
   
   public default DRCRobotModelShapeCollisionSettings getShapeCollisionSettings()
   {
      return new DefaultShapeCollisionSettings();
   }
}
