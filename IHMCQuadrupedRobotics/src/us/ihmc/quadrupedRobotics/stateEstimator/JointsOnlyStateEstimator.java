package us.ihmc.quadrupedRobotics.stateEstimator;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.JointStateUpdater;

public class JointsOnlyStateEstimator implements QuadrupedStateEstimator
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

   @Override
   public void initialize()
   {
      jointStateUpdater.initialize();
      sdfFullRobotModel.updateFrames();
   }

   @Override
   public void enable()
   {
      
   }

   @Override
   public void doControl()
   {
      jointStateUpdater.updateJointState();
      sdfFullRobotModel.updateFrames();
   }

   @Override
   public boolean isFootInContact(RobotQuadrant quadrant)
   {
      return false;
   }

   @Override
   public double getCurrentTime()
   {
      return TimeTools.nanoSecondstoSeconds(sensorOutputMapReadOnly.getTimestamp());
   }

}
