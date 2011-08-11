package us.ihmc.commonWalkingControlModules.sensors;

import us.ihmc.robotSide.RobotSide;

import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;


public class SupportLegAndLegToTrustForVelocity implements LegToTrustForVelocityReadOnly, LegToTrustForVelocityWriteOnly
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SupportLegAndLegToTrustForVelocity");

   private final EnumYoVariable<RobotSide> legToTrustForVelocity = EnumYoVariable.create("legToTrustForVelocity", RobotSide.class, registry);
   private final EnumYoVariable<RobotSide> supportLeg = EnumYoVariable.create("supportLeg", RobotSide.class, registry);
   private final EnumYoVariable<RobotSide> legToUseForCOMOffset = EnumYoVariable.create("legToUseForCOMOffset", RobotSide.class, registry);

   public SupportLegAndLegToTrustForVelocity(YoVariableRegistry yoVariableRegistry)
   {
      yoVariableRegistry.addChild(registry);
   }
   
   public RobotSide getLegToTrustForVelocity()
   {
      return legToTrustForVelocity.getEnumValue();
   }
   
   public RobotSide getSupportLeg()
   {
      return supportLeg.getEnumValue();
   }
   
   public RobotSide getLegToUseForCOMOffset()
   {
      return legToUseForCOMOffset.getEnumValue();
   }

   public void setLegToTrustForVelocity(RobotSide robotSide)
   {
      legToTrustForVelocity.set(robotSide);      
   }
   
   public void setSupportLeg(RobotSide robotSide)
   {
      supportLeg.set(robotSide);      
   }
   
   public void setLegToUseForCOMOffset(RobotSide robotSide)
   {
      legToUseForCOMOffset.set(robotSide);      
   }
}
