package us.ihmc.commonWalkingControlModules.barrierScheduler.context;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;

@SuppressWarnings("serial")
public class AtlasHumanoidRobotContextData extends HumanoidRobotContextData
{
   /**
    * Raw joint measurements for the controller.
    * Set by the estimator.
    */
   private final RawJointSensorDataHolderMap rawJointSensorDataHolderMap;

   public AtlasHumanoidRobotContextData()
   {
      super();
      rawJointSensorDataHolderMap = new RawJointSensorDataHolderMap();
   }

   public AtlasHumanoidRobotContextData(FullHumanoidRobotModel fullRobotModel)
   {
      super(fullRobotModel);
      rawJointSensorDataHolderMap = new RawJointSensorDataHolderMap(fullRobotModel);
   }

   public AtlasHumanoidRobotContextData(HumanoidRobotContextJointData processedJointData, ForceSensorDataHolder forceSensorDataHolder,
                                        CenterOfPressureDataHolder centerOfPressureDataHolder, RobotMotionStatusHolder robotMotionStatusHolder,
                                        LowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList, SensorDataContext sensorDataContext,
                                        RawJointSensorDataHolderMap rawJointSensorDataHolderMap)
   {
      super(processedJointData, forceSensorDataHolder, centerOfPressureDataHolder, robotMotionStatusHolder, jointDesiredOutputList, sensorDataContext);
      this.rawJointSensorDataHolderMap = rawJointSensorDataHolderMap;
   }

   public void set(AtlasHumanoidRobotContextData other)
   {
      copyFrom(other);
   }

   public RawJointSensorDataHolderMap getRawJointSensorDataHolderMap()
   {
      return rawJointSensorDataHolderMap;
   }

   @Override
   public void copyFrom(HumanoidRobotContextData src)
   {
      super.copyFrom(src);

      AtlasHumanoidRobotContextData atlasSrc = (AtlasHumanoidRobotContextData) src;
      rawJointSensorDataHolderMap.set(atlasSrc.rawJointSensorDataHolderMap);
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof AtlasHumanoidRobotContextData)
      {
         AtlasHumanoidRobotContextData other = (AtlasHumanoidRobotContextData) obj;
         if (!rawJointSensorDataHolderMap.equals(other.rawJointSensorDataHolderMap))
            return false;
         return super.equals(other);
      }
      else
      {
         return false;
      }
   }
}
