package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;


public class DesiredFootstepCalculatorFootstepProviderWrapper implements FootstepProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DesiredFootstepCalculator desiredFootstepCalculator;
   private final EnumYoVariable<RobotSide> nextSwingLeg = EnumYoVariable.create("nextSwingLeg", RobotSide.class, registry);
   private final BooleanYoVariable walk = new BooleanYoVariable("walk", registry);
   private Footstep lastPolledFootstep;

   public DesiredFootstepCalculatorFootstepProviderWrapper(final DesiredFootstepCalculator desiredFootstepCalculator, YoVariableRegistry parentRegistry)
   {
      this.desiredFootstepCalculator = desiredFootstepCalculator;
      parentRegistry.addChild(registry);
         
      walk.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (walk.getBooleanValue())
            {
               if (desiredFootstepCalculator != null)
                  desiredFootstepCalculator.initialize();
            }
         }
      });
   }

   public void setNextSwingLeg(RobotSide nextSwingLeg)
   {
      this.nextSwingLeg.set(nextSwingLeg);
   }

   @Override
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

   @Override
   public Footstep peek()
   {
      if (lastPolledFootstep == null)
         return null;
      return desiredFootstepCalculator.predictFootstepAfterDesiredFootstep(nextSwingLeg.getEnumValue(), lastPolledFootstep);
   }

   @Override
   public Footstep peekPeek()
   {
      if (lastPolledFootstep == null)
         return null;
      Footstep nextOne = desiredFootstepCalculator.predictFootstepAfterDesiredFootstep(nextSwingLeg.getEnumValue(), lastPolledFootstep);
      return desiredFootstepCalculator.predictFootstepAfterDesiredFootstep(nextSwingLeg.getEnumValue().getOppositeSide(), nextOne);
   }

   @Override
   public boolean isEmpty()
   {
      return !walk.getBooleanValue() || desiredFootstepCalculator.isDone();
   }

   @Override
   public void notifyComplete(FramePose actualFootPoseInWorld)
   {
   }

   @Override
   public void notifyWalkingComplete()
   {
   }

   public void setWalk(boolean walk)
   {
      this.walk.set(walk);
   }

   @Override
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

   @Override
   public boolean isBlindWalking()
   {
      return true;
   }

   @Override
   public boolean isPaused()
   {
      return false;
   }

   @Override
   public void cancelPlan()
   {
      setWalk(false);
   }
}