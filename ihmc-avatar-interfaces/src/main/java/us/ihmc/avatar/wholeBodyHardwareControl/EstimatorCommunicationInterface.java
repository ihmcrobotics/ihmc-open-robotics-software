package us.ihmc.avatar.wholeBodyHardwareControl;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;

/**
 * General interface for passing measured and desired robot data to and from
 * a physical robot, either through ROS2 or directly via low-level hardware
 * device drivers/managers.
 *
 * @author Stefan Fasano
 */
public interface EstimatorCommunicationInterface
{
   void read(SensorDataContext sensorDataContext);

   void write(HumanoidRobotContextData sensorData, FullHumanoidRobotModel fullRobotModel);

   void start();

   void stop();

   void destroy();

   boolean hasReceivedFirstState();

   default void addSoftEStopListener(YoVariableChangedListener listener)
   {
   }

   boolean hasRobotFaulted();

   void addFaultListener(YoVariableChangedListener listener);
}
