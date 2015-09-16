package us.ihmc.sensorProcessing.sensorProcessors;

import java.util.List;

import us.ihmc.sensorProcessing.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public interface SensorRawOutputMapReadOnly
{
   public long getTimestamp();

   public long getVisionSensorTimestamp();

   public long getSensorHeadPPSTimestamp();
   
   public double getJointPositionRawOutput(OneDoFJoint oneDoFJoint);

   public double getJointVelocityRawOutput(OneDoFJoint oneDoFJoint);
   
   public double getJointAccelerationRawOutput(OneDoFJoint oneDoFJoint);

   public double getJointTauRawOutput(OneDoFJoint oneDoFJoint);
   
   public List<? extends IMUSensorReadOnly> getIMURawOutputs();

   public ForceSensorDataHolderReadOnly getForceSensorRawOutputs();

   public AuxiliaryRobotData getAuxiliaryRobotData();
}
