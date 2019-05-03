package us.ihmc.commonWalkingControlModules.barrierScheduler.context;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;

public class AtlasHumanoidRobotContextData extends HumanoidRobotContextData
{
   private final RawJointSensorDataHolderMap rawJointSensorDataHolderMap;

   protected AtlasHumanoidRobotContextData(HumanoidRobotContextJointData processedJointData, ForceSensorDataHolder forceSensorDataHolder,
                                           CenterOfPressureDataHolder centerOfPressureDataHolder, RobotMotionStatusHolder robotMotionStatusHolder,
                                           LowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList, RawJointSensorDataHolderMap rawJointSensorDataHolderMap)
   {
      super(processedJointData, forceSensorDataHolder, centerOfPressureDataHolder, robotMotionStatusHolder, jointDesiredOutputList);
      this.rawJointSensorDataHolderMap = rawJointSensorDataHolderMap;
   }

   @Override
   public void copyFrom(HumanoidRobotContextData src)
   {
      super.copyFrom(src);

      AtlasHumanoidRobotContextData atlasSrc = (AtlasHumanoidRobotContextData) src;
      rawJointSensorDataHolderMap.set(atlasSrc.rawJointSensorDataHolderMap);
   }
}
