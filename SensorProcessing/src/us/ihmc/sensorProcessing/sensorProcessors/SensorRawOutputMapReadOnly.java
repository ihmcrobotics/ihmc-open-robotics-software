package us.ihmc.sensorProcessing.sensorProcessors;

import java.util.List;

import us.ihmc.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

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

   public AuxiliaryRobotData getAuxiliaryRobotData();
}
