package us.ihmc.quadrupedRobotics.estimator.stateEstimator;

import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimator;
import us.ihmc.stateEstimation.humanoid.DRCStateEstimatorInterface;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.JointStateUpdater;

public class JointsOnlyStateEstimator implements DRCStateEstimatorInterface
{
   private final SDFFullRobotModel sdfFullRobotModel;
   private final SensorOutputMapReadOnly sensorOutputMapReadOnly;
   private final JointStateUpdater jointStateUpdater;



   public JointsOnlyStateEstimator(SDFFullRobotModel sdfFullRobotModel, SensorOutputMapReadOnly sensorOutputMapReadOnly, JointStateUpdater jointStateUpdater)
   {
      this.sdfFullRobotModel = sdfFullRobotModel;
      this.sensorOutputMapReadOnly = sensorOutputMapReadOnly;
      this.jointStateUpdater = jointStateUpdater;
   }

   public void initialize()
   {
      jointStateUpdater.initialize();
      sdfFullRobotModel.updateFrames();
   }

   public void enable()
   {
      
   }

   public void doControl()
   {
      jointStateUpdater.updateJointState();
      sdfFullRobotModel.updateFrames();
   }

   public boolean isFootInContact(RobotQuadrant quadrant)
   {
      return false;
   }

   public double getCurrentTime()
   {
      return TimeTools.nanoSecondstoSeconds(sensorOutputMapReadOnly.getTimestamp());
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
