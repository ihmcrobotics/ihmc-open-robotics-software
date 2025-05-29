package us.ihmc.avatar.hardwareControl;

import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;

public interface HardwareCommunicationInterface
{
   void read(SensorDataContext sensorDataContext);

   void write(JointDesiredOutputListReadOnly jointDesireds);

   boolean hasReceivedFirstState();

   void addSoftEStopListener(YoVariableChangedListener listener);
}
