package us.ihmc.avatar.wholeBodyHardwareControl;

import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;

/**
 * General interface for passing measured and desired robot data to and from
 * a physical robot, either through ROS2 or directly via low-level hardware
 * device drivers/managers
 *
 * @author Stefan Fasano
 */
public interface HardwareCommunicationInterface
{
   void read(SensorDataContext sensorDataContext);

   void write(JointDesiredOutputListReadOnly jointDesireds);

   boolean hasReceivedFirstState();

   void addSoftEStopListener(YoVariableChangedListener listener);
}
