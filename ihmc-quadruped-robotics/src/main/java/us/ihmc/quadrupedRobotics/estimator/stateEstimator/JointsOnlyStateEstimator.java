package us.ihmc.quadrupedRobotics.estimator.stateEstimator;

import gnu.trove.map.TObjectDoubleMap;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.JointStateUpdater;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class JointsOnlyStateEstimator implements StateEstimatorController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final FullRobotModel fullRobotModel;
   private final SensorOutputMapReadOnly sensorOutputMapReadOnly;
   private final JointStateUpdater jointStateUpdater;

   public JointsOnlyStateEstimator(FullRobotModel fullRobotModel, SensorOutputMapReadOnly sensorOutputMapReadOnly)
   {
      this.fullRobotModel = fullRobotModel;
      this.sensorOutputMapReadOnly = sensorOutputMapReadOnly;
      FullInverseDynamicsStructure inverseDynamicsStructure = FullInverseDynamicsStructure.createInverseDynamicStructure(fullRobotModel);
      this.jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, sensorOutputMapReadOnly, null, registry);
   }

   public JointsOnlyStateEstimator(FullRobotModel fullRobotModel, SensorOutputMapReadOnly sensorOutputMapReadOnly, JointStateUpdater jointStateUpdater)
   {
      this.fullRobotModel = fullRobotModel;
      this.sensorOutputMapReadOnly = sensorOutputMapReadOnly;
      this.jointStateUpdater = jointStateUpdater;
   }

   @Override
   public void initialize()
   {
      jointStateUpdater.initialize();
      fullRobotModel.updateFrames();
   }

   public void enable()
   {

   }

   @Override
   public void doControl()
   {
      jointStateUpdater.updateJointState();
      fullRobotModel.updateFrames();
   }

   public boolean isFootInContact(RobotQuadrant quadrant)
   {
      return false;
   }

   public double getCurrentTime()
   {
      return Conversions.nanosecondsToSeconds(sensorOutputMapReadOnly.getWallTime());
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return null;
   }

   @Override
   public String getDescription()
   {
      return null;
   }

   @Override
   public void requestStateEstimatorMode(StateEstimatorMode operatingMode)
   {
   }

   @Override
   public void initializeEstimator(RigidBodyTransform rootJointTransform, TObjectDoubleMap<String> jointPositions)
   {
   }

}
