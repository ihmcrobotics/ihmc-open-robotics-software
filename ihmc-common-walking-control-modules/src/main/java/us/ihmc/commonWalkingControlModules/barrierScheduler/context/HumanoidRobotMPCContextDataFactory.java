package us.ihmc.commonWalkingControlModules.barrierScheduler.context;

import us.ihmc.tools.factories.FactoryTools;

public class HumanoidRobotMPCContextDataFactory extends HumanoidRobotContextDataFactory
{
   public HumanoidRobotMPCContextData createHumanoidRobotMPCContextData()
   {
      FactoryTools.checkAllFactoryFieldsAreSet(this);

      return new HumanoidRobotMPCContextData(processedJointData.get(),
                                             forceSensorDataHolder.get(),
                                             centerOfMassDataHolder.get(),
                                             centerOfPressureDataHolder.get(),
                                             robotMotionStatusHolder.get(),
                                             jointDesiredOutputList.get(),
                                             sensorDataContext.get());
   }
}
