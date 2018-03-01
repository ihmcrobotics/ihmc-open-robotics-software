package us.ihmc.sensorProcessing.sensorProcessors;

import java.util.List;

import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.AtlasAuxiliaryRobotData;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

public interface SensorRawOutputMapReadOnly extends SensorTimestampHolder
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

   public AtlasAuxiliaryRobotData getAuxiliaryRobotData();
}
