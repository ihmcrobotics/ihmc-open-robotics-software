package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;


public class KneeDamperControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("KneeDamperControlModule");
   private final ProcessedSensorsInterface processedSensors;
   private final DoubleYoVariable bKneeDamping = new DoubleYoVariable("bKneeDamping", registry);
   private final DoubleYoVariable ffKneeToStraighten = new DoubleYoVariable("ffKneeToStraighten", registry);

   public KneeDamperControlModule(ProcessedSensorsInterface processedSensors, YoVariableRegistry parentRegistry)
   {
      this.processedSensors = processedSensors;
      parentRegistry.addChild(registry);
   }

   public void addKneeDamping(LegTorques supportLegTorquesToPack)
   {
      RobotSide robotSide = supportLegTorquesToPack.getRobotSide();

      double qd = processedSensors.getLegJointVelocity(robotSide, LegJointName.KNEE);
      double kneeDampingTorque = -qd * bKneeDamping.getDoubleValue() - ffKneeToStraighten.getDoubleValue();

      supportLegTorquesToPack.addTorque(LegJointName.KNEE, kneeDampingTorque);
   }
   
   public void setParametersForR2()
   {
      bKneeDamping.set(50.0);    // 100.0);
      ffKneeToStraighten.set(0.0);    // 10.0);
   }
   
   public void setParametersForM2V2()
   {
      bKneeDamping.set(6.0); 
      ffKneeToStraighten.set(0.0);    // 10.0);
   }
}
