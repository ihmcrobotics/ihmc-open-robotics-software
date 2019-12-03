package us.ihmc.sensorProcessing.sensorProcessors;

import java.util.List;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

public interface SensorOutputMapReadOnly extends SensorTimestampHolder
{
   OneDoFJointStateReadOnly getOneDoFJointOutput(OneDoFJointBasics oneDoFJoint);

   List<? extends OneDoFJointStateReadOnly> getOneDoFJointOutputs();

   List<? extends IMUSensorReadOnly> getIMUOutputs();

   ForceSensorDataHolderReadOnly getForceSensorOutputs();
}
