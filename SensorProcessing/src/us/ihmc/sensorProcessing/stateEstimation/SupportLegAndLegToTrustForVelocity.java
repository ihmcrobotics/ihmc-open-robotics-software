package us.ihmc.sensorProcessing.stateEstimation;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;



public class SupportLegAndLegToTrustForVelocity implements LegToTrustForVelocityReadOnly, LegToTrustForVelocityWriteOnly
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SupportLegAndLegToTrustForVelocity");

   private final SideDependentList<BooleanYoVariable> legsToTrustForVelocity = new SideDependentList<BooleanYoVariable>();
   private final EnumYoVariable<RobotSide> supportLeg = EnumYoVariable.create("supportLeg", "", RobotSide.class, registry, true);
   private final EnumYoVariable<RobotSide> legToUseForCOMOffset = EnumYoVariable.create("legToUseForCOMOffset", "", RobotSide.class, registry, true);

   public SupportLegAndLegToTrustForVelocity(YoVariableRegistry parentRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         legsToTrustForVelocity.put(robotSide, new BooleanYoVariable("trust" + robotSide.getCamelCaseNameForMiddleOfExpression() + "LegForVelocity", registry));
      }

      parentRegistry.addChild(registry);
   }

   public boolean isLegTrustedForVelocity(RobotSide robotSide)
   {
      return legsToTrustForVelocity.get(robotSide).getBooleanValue();
   }

   public RobotSide getSupportLeg()
   {
      return supportLeg.getEnumValue();
   }

   public RobotSide getLegToUseForCOMOffset()
   {
      return legToUseForCOMOffset.getEnumValue();
   }

   public void setLegToTrustForVelocity(RobotSide robotSide, boolean trust)
   {
      legsToTrustForVelocity.get(robotSide).set(trust);
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
