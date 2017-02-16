package us.ihmc.quadrupedRobotics.estimator.stateEstimator;

import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;

import us.ihmc.commons.Conversions;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimator;
import us.ihmc.stateEstimation.humanoid.DRCStateEstimatorInterface;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.JointStateUpdater;

public class JointsOnlyStateEstimator implements DRCStateEstimatorInterface
{
   private final FullRobotModel fullRobotModel;
   private final SensorOutputMapReadOnly sensorOutputMapReadOnly;
   private final JointStateUpdater jointStateUpdater;



   public JointsOnlyStateEstimator(FullRobotModel fullRobotModel, SensorOutputMapReadOnly sensorOutputMapReadOnly, JointStateUpdater jointStateUpdater)
   {
      this.fullRobotModel = fullRobotModel;
      this.sensorOutputMapReadOnly = sensorOutputMapReadOnly;
      this.jointStateUpdater = jointStateUpdater;
   }

   public void initialize()
   {
      jointStateUpdater.initialize();
      fullRobotModel.updateFrames();
   }

   public void enable()
   {

   }

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
      return Conversions.nanoSecondstoSeconds(sensorOutputMapReadOnly.getTimestamp());
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
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
   public StateEstimator getStateEstimator()
   {
      return null;
   }

   @Override
   public void initializeEstimatorToActual(Tuple3d initialCoMPosition, Quat4d initialEstimationLinkOrientation)
   {

   }

}
