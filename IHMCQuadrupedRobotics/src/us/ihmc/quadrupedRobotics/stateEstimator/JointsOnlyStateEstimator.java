package us.ihmc.quadrupedRobotics.stateEstimator;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.JointStateUpdater;

public class JointsOnlyStateEstimator
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

}
