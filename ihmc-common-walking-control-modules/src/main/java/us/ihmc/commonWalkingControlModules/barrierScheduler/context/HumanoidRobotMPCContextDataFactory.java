package us.ihmc.commonWalkingControlModules.barrierScheduler.context;

import us.ihmc.tools.factories.FactoryTools;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class HumanoidRobotMPCContextDataFactory extends HumanoidRobotContextDataFactory
{
   public HumanoidRobotMPCContextData createHumanoidRobotContextData()
   {
      FactoryTools.checkAllFactoryFieldsAreSet(this);

      return new HumanoidRobotMPCContextData(processedJointData.get(),
                                             forceSensorDataHolder.get(),
                                             centerOfMassDataHolder.get(),
                                             centerOfPressureDataHolder.get(),
                                             robotMotionStatusHolder.get(),
                                             jointDesiredOutputList.get(),
                                             sensorDataContext.get(),
                                             wbccJointDesiredOutputList.get(),
                                             controllerCoreCommandDataHolder.get(),
                                             controllerCoreOutputDataHolder.get());
   }
}