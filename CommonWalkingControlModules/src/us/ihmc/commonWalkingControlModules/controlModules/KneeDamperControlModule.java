package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

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

      double kneeTorque = supportLegTorquesToPack.getTorque(LegJointName.KNEE);
      kneeTorque -= processedSensors.getLegJointVelocity(robotSide, LegJointName.KNEE) * bKneeDamping.getDoubleValue();

      kneeTorque -= ffKneeToStraighten.getDoubleValue();

      supportLegTorquesToPack.setTorque(LegJointName.KNEE, kneeTorque);
   }
   
   public void setParametersForR2()
   {
      bKneeDamping.set(50.0);    // 100.0);
      ffKneeToStraighten.set(0.0);    // 10.0);
   }
   
   public void setParametersForM2V2()
   {
      // TODO: tune
      bKneeDamping.set(50.0);    // 100.0);
      ffKneeToStraighten.set(0.0);    // 10.0);
   }
}
