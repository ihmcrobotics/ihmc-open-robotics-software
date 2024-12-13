package us.ihmc.commonWalkingControlModules.barrierScheduler.context;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.RequiredFactoryField;

public class HumanoidRobotMPCContextDataFactory extends HumanoidRobotContextDataFactory
{
   protected final RequiredFactoryField<LowLevelOneDoFJointDesiredDataHolder> mpcControllerDesiredOutputList = new RequiredFactoryField<>(
         "mpcControllerDesiredOutputList");

   public HumanoidRobotMPCContextData createHumanoidRobotMPCContextData()
   {
      FactoryTools.checkAllFactoryFieldsAreSet(this);

      return new HumanoidRobotMPCContextData(processedJointData.get(),
                                             forceSensorDataHolder.get(),
                                             centerOfMassDataHolder.get(),
                                             centerOfPressureDataHolder.get(),
                                             robotMotionStatusHolder.get(),
                                             jointDesiredOutputList.get(),
                                             sensorDataContext.get(),
                                             mpcControllerDesiredOutputList.get());
   }

   public void setMpcControllerDesiredOutputList(LowLevelOneDoFJointDesiredDataHolder value)
   {
      mpcControllerDesiredOutputList.set(value);
   }
}
