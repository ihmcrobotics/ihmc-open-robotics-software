package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.humanoidRobot.footstep.Footstep;


public class DesiredFootstepCalculatorFootstepProviderWrapper implements FootstepProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DesiredFootstepCalculator desiredFootstepCalculator;
   private final EnumYoVariable<RobotSide> nextSwingLeg = EnumYoVariable.create("nextSwingLeg", RobotSide.class, registry);
   private final BooleanYoVariable walk = new BooleanYoVariable("walk", registry);
   private Footstep lastPolledFootstep;

   public DesiredFootstepCalculatorFootstepProviderWrapper(DesiredFootstepCalculator desiredFootstepCalculator, YoVariableRegistry parentRegistry)
   {
      this.desiredFootstepCalculator = desiredFootstepCalculator;
      parentRegistry.addChild(registry);
   }

   public void setNextSwingLeg(RobotSide nextSwingLeg)
   {
      this.nextSwingLeg.set(nextSwingLeg);
   }

   public Footstep poll()
   {
      Footstep ret = null;
      if (!isEmpty())
      {
         RobotSide supportLeg = nextSwingLeg.getEnumValue().getOppositeSide();
         desiredFootstepCalculator.initializeDesiredFootstep(supportLeg);
         ret = desiredFootstepCalculator.updateAndGetDesiredFootstep(supportLeg);
         nextSwingLeg.set(supportLeg);
      }

      lastPolledFootstep = ret;

      return ret;
   }

   public Footstep peek()
   {
      return desiredFootstepCalculator.predictFootstepAfterDesiredFootstep(nextSwingLeg.getEnumValue(), lastPolledFootstep);
   }

   public Footstep peekPeek()
   {
      Footstep nextOne = desiredFootstepCalculator.predictFootstepAfterDesiredFootstep(nextSwingLeg.getEnumValue(), lastPolledFootstep);
      return desiredFootstepCalculator.predictFootstepAfterDesiredFootstep(nextSwingLeg.getEnumValue().getOppositeSide(), nextOne);
   }

   public boolean isEmpty()
   {
      return !walk.getBooleanValue() || desiredFootstepCalculator.isDone();
   }

   public void notifyComplete()
   {
   }

   public void setWalk(boolean walk)
   {
      this.walk.set(walk);
   }

   public int getNumberOfFootstepsToProvide()
   {
      if (walk.getBooleanValue())
      {
         return Integer.MAX_VALUE;
      }
      else
      {
         return 0;
      }
   }

   public boolean isBlindWalking()
   {
      return true;
   }
}