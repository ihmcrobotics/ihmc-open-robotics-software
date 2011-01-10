package us.ihmc.commonWalkingControlModules.sensors;

import us.ihmc.commonWalkingControlModules.RobotSide;

import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;


public class SupportLegAndLegToTrustForVelocity
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SupportLegAndLegToTrustForVelocity");

   public final EnumYoVariable<RobotSide> legToTrustForVelocity = EnumYoVariable.create("legToTrustForVelocity", RobotSide.class, registry);
   public final EnumYoVariable<RobotSide> supportLeg = EnumYoVariable.create("supportLeg", RobotSide.class, registry);
   public final EnumYoVariable<RobotSide> legToUseForCOMOffset = EnumYoVariable.create("legToUseForCOMOffset", RobotSide.class, registry);

   public SupportLegAndLegToTrustForVelocity(YoVariableRegistry yoVariableRegistry)
   {
      yoVariableRegistry.addChild(registry);
   }
}
