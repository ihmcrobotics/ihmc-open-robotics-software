package us.ihmc.quadrupedRobotics.sensorProcessing.simulatedSensors;

import java.util.List;

import us.ihmc.quadrupedRobotics.sensorProcessing.SimulatedRobotTimeProvider;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

public class ControllerOutputMapReadOnly implements SensorOutputMapReadOnly
{
   /**
    * the purpose of this class is to read the q, qd and qdd desired of the oneDofJoints and to return it as the sensor output 
    */
   SimulatedRobotTimeProvider timeProvider;

   public ControllerOutputMapReadOnly(SimulatedRobotTimeProvider timeProvider)
   {
      this.timeProvider = timeProvider;
   }

   @Override
   public long getTimestamp()
   {
      return timeProvider.getTimestamp();
   }

   @Override
   public long getVisionSensorTimestamp()
   {
      return getTimestamp();
   }

   @Override
   public long getSensorHeadPPSTimestamp()
   {
      return getTimestamp();
   }

   @Override
   public double getJointPositionProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return oneDoFJoint.getqDesired();
   }

   @Override
   public double getJointVelocityProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return oneDoFJoint.getQdDesired();
   }

   @Override
   public double getJointAccelerationProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return oneDoFJoint.getQddDesired();
   }

   @Override
   public double getJointTauProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public boolean isJointEnabled(OneDoFJoint oneDoFJoint)
   {
      return oneDoFJoint.isEnabled();
   }

   @Override
   public List<? extends IMUSensorReadOnly> getIMUProcessedOutputs()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public ForceSensorDataHolderReadOnly getForceSensorProcessedOutputs()
   {
      // TODO Auto-generated method stub
      return null;
   }

}
