package us.ihmc.avatar.drcRobot;

import java.util.LinkedHashMap;

import org.apache.commons.lang3.tuple.ImmutablePair;

import com.jme3.math.Transform;

import us.ihmc.avatar.handControl.HandCommandManager;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.FootstepPlanningParameterization;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.FootstepSnappingParameters;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.wholeBodyController.DRCHandType;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.SimulatedWholeBodyControllerParameters;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;

public interface DRCRobotModel extends SimulatedWholeBodyControllerParameters
{
   enum RobotTarget
   {
      SCS, GAZEBO, REAL_ROBOT, HEAD_ON_A_STICK
   }

   // TODO: RobotBoundingBoxes.java

   // public abstract boolean isRunningOnRealRobot();

   public abstract FootstepPlanningParameterization getFootstepParameters();

   public abstract StateEstimatorParameters getStateEstimatorParameters();

   public abstract DRCRobotPhysicalProperties getPhysicalProperties();

   public abstract DRCRobotJointMap getJointMap();

   public abstract DRCRobotSensorInformation getSensorInformation();

   public abstract DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw);

   public abstract void setEnableJointDamping(boolean enableJointDamping);

   public abstract boolean getEnableJointDamping();

   public abstract HandModel getHandModel();

   public abstract Transform getJmeTransformWristToHand(RobotSide side);

   public abstract RigidBodyTransform getTransform3dWristToHand(RobotSide side);

   public abstract double getSimulateDT();

   public abstract double getEstimatorDT();

   public abstract double getStandPrepAngle(String jointName);

   public abstract DRCROSPPSTimestampOffsetProvider getPPSTimestampOffsetProvider();

   public abstract DRCSensorSuiteManager getSensorSuiteManager();

   public abstract SideDependentList<HandCommandManager> createHandCommandManager();

   public abstract MultiThreadedRobotControlElement createSimulatedHandController(FloatingRootJointRobot simulatedRobot,
         ThreadDataSynchronizerInterface threadDataSynchronizer, HumanoidGlobalDataProducer globalDataProducer,
         CloseableAndDisposableRegistry closeableAndDisposableRegistry);

   public abstract DRCHandType getDRCHandType();

   public abstract LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>> getSliderBoardControlledNeckJointsWithLimits();

   public abstract SideDependentList<LinkedHashMap<String, ImmutablePair<Double, Double>>> getSliderBoardControlledFingerJointsWithLimits();

   public abstract LogSettings getLogSettings();

   public abstract LogModelProvider getLogModelProvider();

   public abstract String getSimpleRobotName();

   public abstract CollisionBoxProvider getCollisionBoxProvider();

   public abstract FootstepSnappingParameters getSnappingParameters();
}
