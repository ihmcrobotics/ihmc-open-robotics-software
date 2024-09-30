package us.ihmc.commonWalkingControlModules.barrierScheduler.context;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.RequiredFactoryField;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class HumanoidRobotContextDataFactory
{
   protected final RequiredFactoryField<HumanoidRobotContextJointData> processedJointData = new RequiredFactoryField<>("processedJointData");
   protected final RequiredFactoryField<ForceSensorDataHolder> forceSensorDataHolder = new RequiredFactoryField<>("forceSensorDataHolder");
   protected final RequiredFactoryField<CenterOfMassDataHolder> centerOfMassDataHolder = new RequiredFactoryField<>("centerOfMassDataHolder");
   protected final RequiredFactoryField<CenterOfPressureDataHolder> centerOfPressureDataHolder = new RequiredFactoryField<>("centerOfPressureDataHolder");
   protected final RequiredFactoryField<RobotMotionStatusHolder> robotMotionStatusHolder = new RequiredFactoryField<>("robotMotionStatusHolder");
   protected final RequiredFactoryField<LowLevelOneDoFJointDesiredDataHolder> jointDesiredOutputList = new RequiredFactoryField<>("jointDesiredOutputList");
   protected final RequiredFactoryField<SensorDataContext> sensorDataContext = new RequiredFactoryField<>("sensorDataContext");
   protected final RequiredFactoryField<LowLevelOneDoFJointDesiredDataHolder> wbccJointDesiredOutputList = new RequiredFactoryField<>("wholeBodyControllerCoreJointDesiredOutputList");


   public HumanoidRobotContextData createHumanoidRobotContextData()
   {
      FactoryTools.checkAllFactoryFieldsAreSet(this);

      return new HumanoidRobotContextData(processedJointData.get(),
                                          forceSensorDataHolder.get(),
                                          centerOfMassDataHolder.get(),
                                          centerOfPressureDataHolder.get(),
                                          robotMotionStatusHolder.get(),
                                          jointDesiredOutputList.get(),
                                          sensorDataContext.get(),
                                          wbccJointDesiredOutputList.get());
   }

   public void setProcessedJointData(HumanoidRobotContextJointData value)
   {
      processedJointData.set(value);
   }

   public void setForceSensorDataHolder(ForceSensorDataHolder value)
   {
      forceSensorDataHolder.set(value);
   }

   public void setCenterOfMassDataHolder(CenterOfMassDataHolder value)
   {
      centerOfMassDataHolder.set(value);
   }

   public void setCenterOfPressureDataHolder(CenterOfPressureDataHolder value)
   {
      centerOfPressureDataHolder.set(value);
   }

   public void setRobotMotionStatusHolder(RobotMotionStatusHolder value)
   {
      robotMotionStatusHolder.set(value);
   }

   public void setJointDesiredOutputList(LowLevelOneDoFJointDesiredDataHolder value)
   {
      jointDesiredOutputList.set(value);
   }

   public void setWBCCJointDesiredOutputList(LowLevelOneDoFJointDesiredDataHolder value)
   {
      wbccJointDesiredOutputList.set(value);
   }
   public void setSensorDataContext(SensorDataContext value)
   {
      sensorDataContext.set(value);
   }
}
