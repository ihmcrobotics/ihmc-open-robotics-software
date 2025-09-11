package us.ihmc.avatar.wholeBodyHardwareControl;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;

/**
 * General interface for receiving the measured data and then publishing the estimated robot data from a physical robot.
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
