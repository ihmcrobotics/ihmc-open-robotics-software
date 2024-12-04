package us.ihmc.commonWalkingControlModules.barrierScheduler.context;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.ControllerCoreOutputDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.RequiredFactoryField;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class HumanoidRobotMPCContextDataFactory extends HumanoidRobotContextDataFactory
{
   protected final RequiredFactoryField<LowLevelOneDoFJointDesiredDataHolder> wbccJointDesiredOutputList = new RequiredFactoryField<>(
         "wholeBodyControllerCoreJointDesiredOutputList");
   protected final RequiredFactoryField<ControllerCoreOutputDataHolder> controllerCoreOutputDataHolder = new RequiredFactoryField<>("controllerCoreDataHolder");
   protected final RequiredFactoryField<ControllerCoreCommandDataHolder> controllerCoreCommandDataHolder = new RequiredFactoryField<>(
         "controllerCoreCommandDataHolder");

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


   public void setWBCCJointDesiredOutputList(LowLevelOneDoFJointDesiredDataHolder value)
   {
      wbccJointDesiredOutputList.set(value);
   }

   public void setControllerCoreOutputDataHolder(ControllerCoreOutputDataHolder value)
   {
      controllerCoreOutputDataHolder.set(value);
   }

   public void setControllerCoreCommandDataHolder(ControllerCoreCommandDataHolder value)
   {
      controllerCoreCommandDataHolder.set(value);
   }
}