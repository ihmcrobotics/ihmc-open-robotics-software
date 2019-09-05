package us.ihmc.commonWalkingControlModules.barrierScheduler.context;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
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
   protected final RequiredFactoryField<CenterOfPressureDataHolder> centerOfPressureDataHolder = new RequiredFactoryField<>("centerOfPressureDataHolder");
   protected final RequiredFactoryField<RobotMotionStatusHolder> robotMotionStatusHolder = new RequiredFactoryField<>("robotMotionStatusHolder");
   protected final RequiredFactoryField<LowLevelOneDoFJointDesiredDataHolder> jointDesiredOutputList = new RequiredFactoryField<>("jointDesiredOutputList");
   protected final RequiredFactoryField<SensorDataContext> sensorDataContext = new RequiredFactoryField<>("sensorDataContext");

   public HumanoidRobotContextData createHumanoidRobotContextData()
   {
      FactoryTools.checkAllFactoryFieldsAreSet(this);

      return new HumanoidRobotContextData(processedJointData.get(), forceSensorDataHolder.get(), centerOfPressureDataHolder.get(),
                                          robotMotionStatusHolder.get(), jointDesiredOutputList.get(), sensorDataContext.get());
   }

   public void setProcessedJointData(HumanoidRobotContextJointData value)
   {
      this.processedJointData.set(value);
   }

   public void setForceSensorDataHolder(ForceSensorDataHolder value)
   {
      this.forceSensorDataHolder.set(value);
   }

   public void setCenterOfPressureDataHolder(CenterOfPressureDataHolder value)
   {
      this.centerOfPressureDataHolder.set(value);
   }

   public void setRobotMotionStatusHolder(RobotMotionStatusHolder value)
   {
      this.robotMotionStatusHolder.set(value);
   }

   public void setJointDesiredOutputList(LowLevelOneDoFJointDesiredDataHolder value)
   {
      this.jointDesiredOutputList.set(value);
   }

   public void setSensorDataContext(SensorDataContext value)
   {
      this.sensorDataContext.set(value);
   }
}
