package us.ihmc.realtime.barrierScheduler.context;

import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.RequiredFactoryField;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class HumanoidRobotContextDataFactory
{
   private final RequiredFactoryField<HumanoidRobotContextJointData> rawJointData = new RequiredFactoryField<>("rawJointData");
   private final RequiredFactoryField<HumanoidRobotContextJointData> processedJointData = new RequiredFactoryField<>("processedJointData");

   private final RequiredFactoryField<ForceSensorDataHolder> forceSensorDataHolder = new RequiredFactoryField<>("forceSensorDataHolder");
   private final RequiredFactoryField<CenterOfPressureDataHolder> centerOfPressureDataHolder = new RequiredFactoryField<>("centerOfPressureDataHolder");
   private final RequiredFactoryField<RobotMotionStatusHolder> robotMotionStatusHolder = new RequiredFactoryField<>("robotMotionStatusHolder");
   private final RequiredFactoryField<JointDesiredOutputList> jointDesiredOutputList = new RequiredFactoryField<>("jointDesiredOutputList");

   public HumanoidRobotContextData createHumanoidRobotContextData()
   {
      FactoryTools.checkAllFactoryFieldsAreSet(this);

      return new HumanoidRobotContextData(rawJointData.get(), processedJointData.get(), forceSensorDataHolder.get(),
                                          centerOfPressureDataHolder.get(), robotMotionStatusHolder.get(), jointDesiredOutputList.get());
   }

   public void setRawJointData(HumanoidRobotContextJointData value)
   {
      this.rawJointData.set(value);
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

   public void setJointDesiredOutputList(JointDesiredOutputList value)
   {
      this.jointDesiredOutputList.set(value);
   }
}
