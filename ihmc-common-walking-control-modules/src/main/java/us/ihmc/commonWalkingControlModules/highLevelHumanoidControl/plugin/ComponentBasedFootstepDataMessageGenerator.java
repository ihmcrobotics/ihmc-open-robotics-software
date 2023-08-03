package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.DesiredTurningVelocityProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.DesiredVelocityProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ComponentBasedFootstepDataMessageGenerator implements HumanoidSteppingPlugin
{
   private final YoRegistry registry;
   private final ContinuousStepGenerator continuousStepGenerator;
   private final List<Updatable> updatables;

   public ComponentBasedFootstepDataMessageGenerator(ContinuousStepGenerator continuousStepGenerator, List<Updatable> updatables, YoRegistry registry)
   {
      this.registry = registry;
      this.continuousStepGenerator = continuousStepGenerator;
      this.updatables = updatables;
   }

   @Override
   public void update(double time)
   {
      for (int i = 0; i < updatables.size(); i++)
      {
         updatables.get(i).update(time);
      }
      continuousStepGenerator.update(time);
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }

   @Override
   public void setFootstepAdjustment(FootstepAdjustment footstepAdjustment)
   {
      continuousStepGenerator.setFootstepAdjustment(footstepAdjustment);
   }

   /**
    * Sets the provider to use for getting every tick the desired turning velocity.
    *
    * @param desiredTurningVelocityProvider provider for obtaining the desired turning velocity.
    */
   public void setDesiredTurningVelocityProvider(DesiredTurningVelocityProvider desiredTurningVelocityProvider)
   {
      continuousStepGenerator.setDesiredTurningVelocityProvider(desiredTurningVelocityProvider);
   }

   /**
    * Sets the provider to use for getting every tick the desired forward/lateral velocity.
    *
    * @param desiredVelocityProvider provider for obtaining the desired forward/lateral velocity.
    */
   public void setDesiredVelocityProvider(DesiredVelocityProvider desiredVelocityProvider)
   {
      continuousStepGenerator.setDesiredVelocityProvider(desiredVelocityProvider);
   }

   /**
    * Sets a provider that is to be used to update the state of {@link #walk} internally on each call
    * to {@link #update(double)}.
    *
    * @param walkInputProvider the provider used to determine whether to walk or not walk.
    */
   public void setWalkInputProvider(BooleanProvider walkInputProvider)
   {
      continuousStepGenerator.setWalkInputProvider(walkInputProvider);
   }

   /**
    * Sets a provider that is to be used to update the desired swing height of each foot internally
    * on each call to {@link #update(double)}
    *
    * @param swingHeightInputProvider the provider used to set the swing height
    */
   public void setSwingHeightInputProvider(DoubleProvider swingHeightInputProvider)
   {
      continuousStepGenerator.setSwingHeightInputProvider(swingHeightInputProvider);
   }

   /**
    * Sets a provider that is used to update the swing height
    * @return
    */

   public ContinuousStepGenerator getContinuousStepGenerator()
   {
      return continuousStepGenerator;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return continuousStepGenerator.getSCS2YoGraphics();
   }
}
