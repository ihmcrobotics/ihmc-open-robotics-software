package us.ihmc.commonWalkingControlModules.barrierScheduler.context;

import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.RequiredFactoryField;

public class AtlasHumanoidRobotContextDataFactory extends HumanoidRobotContextDataFactory
{
   private final RequiredFactoryField<RawJointSensorDataHolderMap> rawJointData = new RequiredFactoryField<>("rawJointData");

   @Override
   public AtlasHumanoidRobotContextData createHumanoidRobotContextData()
   {
      // TODO: this does not check fields of super factory.
      FactoryTools.checkAllFactoryFieldsAreSet(this);

      return new AtlasHumanoidRobotContextData(processedJointData.get(), forceSensorDataHolder.get(), centerOfPressureDataHolder.get(),
                                               robotMotionStatusHolder.get(), jointDesiredOutputList.get(), sensorDataContext.get(), rawJointData.get());
   }

   public void setRawJointData(RawJointSensorDataHolderMap value)
   {
      this.rawJointData.set(value);
   }
}
