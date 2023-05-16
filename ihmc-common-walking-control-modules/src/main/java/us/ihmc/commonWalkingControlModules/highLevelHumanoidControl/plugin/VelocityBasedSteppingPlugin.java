package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.*;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;

public class VelocityBasedSteppingPlugin implements HumanoidSteppingPlugin
{
   private final static Vector2DReadOnly zero2D = new Vector2D();

   private DesiredVelocityProvider desiredVelocityProvider = () -> zero2D;
   private DesiredTurningVelocityProvider desiredTurningVelocityProvider = () -> 0.0;
   private DirectionalControlMessenger directionalControlMessenger;
   private StopWalkingMessenger stopWalkingMessenger;
   private StartWalkingMessenger startWalkingMessenger;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final String variableNameSuffix = "FWJ";

   private final YoBoolean ignoreWalkInputProvider = new YoBoolean("ignoreWalkInputProvider" + variableNameSuffix, registry);
   private final YoBoolean walk = new YoBoolean("walk" + variableNameSuffix, registry);
   private final YoBoolean walkPreviousValue = new YoBoolean("walkPreviousValue" + variableNameSuffix, registry);
   private final YoVelocityBasedSteppingParameters inputParameters = new YoVelocityBasedSteppingParameters("FSG", registry);

   private final YoDouble desiredTurningVelocity = new YoDouble("desiredTurningVelocity" + variableNameSuffix, registry);
   private final YoVector2D desiredVelocity = new YoVector2D("desiredVelocity" + variableNameSuffix, registry);

   private final YoInteger numberOfTicksBeforeSubmittingCommands = new YoInteger("numberOfTicksBeforeSubmittingFootsteps" + variableNameSuffix, registry);

   private BooleanProvider walkInputProvider;
   private DoubleProvider swingHeightInputProvider;

   private final List<Updatable> updatables;

   public VelocityBasedSteppingPlugin(List<Updatable> updatables)
   {
      this.updatables = updatables;
   }

   public void setInputParameters(VelocityBasedSteppingParameters parameters)
   {
      inputParameters.set(parameters);
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }

   private int counter = 0;

   @Override
   public void update(double time)
   {
      for (int i = 0; i < updatables.size(); i++)
      {
         updatables.get(i).update(time);
      }

      if (!ignoreWalkInputProvider.getBooleanValue() && walkInputProvider != null)
         walk.set(walkInputProvider.getValue());

      if (!walk.getValue())
      {
         if (stopWalkingMessenger != null && walk.getValue() != walkPreviousValue.getValue())
         {
            stopWalkingMessenger.submitStopWalkingRequest();
         }

         walkPreviousValue.set(false);
         return;
      }
      else if (startWalkingMessenger != null && walk.getValue() != walkPreviousValue.getValue())
      {
         startWalkingMessenger.submitStartWalkingRequest();
      }

      if (walk.getValue() != walkPreviousValue.getValue())
      {
         counter = numberOfTicksBeforeSubmittingCommands.getValue(); // To make footsteps being sent right away.
      }

      Vector2DReadOnly desiredVelocity = desiredVelocityProvider.getDesiredVelocity();
      double desiredVelocityX = desiredVelocity.getX();
      double desiredVelocityY = desiredVelocity.getY();
      double turningVelocity = desiredTurningVelocityProvider.getTurningVelocity();

      if (desiredVelocityProvider.isUnitVelocity())
      {
         if (desiredVelocityX > 0)
            desiredVelocityX = inputParameters.getMaxDesiredForwardVelocity() * MathTools.clamp(desiredVelocityX, 1.0);
         else
            desiredVelocityX = inputParameters.getMaxDesiredBackwardVelocity() * MathTools.clamp(desiredVelocityX, 1.0);

         desiredVelocityY = inputParameters.getMaxDesiredLateralVelocity() * MathTools.clamp(desiredVelocityY, 1.0);
      }
      else
      {
         desiredVelocityX = MathTools.clamp(desiredVelocityX, -inputParameters.getMaxDesiredBackwardVelocity(), inputParameters.getMaxDesiredForwardVelocity());
         desiredVelocityY = MathTools.clamp(desiredVelocityY, inputParameters.getMaxDesiredLateralVelocity());
      }

      if (desiredTurningVelocityProvider.isUnitVelocity())
      {
         turningVelocity = inputParameters.getMaxDesiredTurningVelocity() * MathTools.clamp(turningVelocity, 1.0);
      }
      else
      {
         turningVelocity = MathTools.clamp(turningVelocity, inputParameters.getMaxDesiredTurningVelocity());
      }

      this.desiredVelocity.set(desiredVelocityX, desiredVelocityY);
      this.desiredTurningVelocity.set(turningVelocity);

      if (walk.getValue() && directionalControlMessenger != null)
      {
         if (counter >= numberOfTicksBeforeSubmittingCommands.getValue())
         {
            directionalControlMessenger.submitDirectionalControlRequest(desiredVelocityX, desiredVelocityY, turningVelocity);
            directionalControlMessenger.submitGaitParameters(swingHeightInputProvider.getValue(), Double.NaN, Double.NaN);
            counter = 0;
         }
         else
         {
            counter++;
         }
      }

      walkPreviousValue.set(walk.getBooleanValue());
   }

   @Override
   public void setFootstepAdjustment(FootstepAdjustment footstepAdjustment)
   {
   }

   /**
    * Sets a provider that is to be used to update the state of {@link #walk} internally on each call
    * to {@link #update(double)}.
    *
    * @param walkInputProvider the provider used to determine whether to walk or not walk.
    */
   public void setWalkInputProvider(BooleanProvider walkInputProvider)
   {
      this.walkInputProvider = walkInputProvider;
   }

   /**
    * Sets a provider that is to be used to update the desired swing height of each foot internally
    * on each call to {@link #update(double)}
    *
    * @param swingHeightInputProvider the provider used to set the swing height
    */
   public void setSwingHeightInputProvider(DoubleProvider swingHeightInputProvider)
   {
      this.swingHeightInputProvider = swingHeightInputProvider;
   }

   /**
    * Sets the provider to use for getting every tick the desired turning velocity.
    *
    * @param desiredTurningVelocityProvider provider for obtaining the desired turning velocity.
    */
   public void setDesiredTurningVelocityProvider(DesiredTurningVelocityProvider desiredTurningVelocityProvider)
   {
      this.desiredTurningVelocityProvider = desiredTurningVelocityProvider;
   }

   /**
    * Sets the provider to use for getting every tick the desired forward/lateral velocity.
    *
    * @param desiredVelocityProvider provider for obtaining the desired forward/lateral velocity.
    */
   public void setDesiredVelocityProvider(DesiredVelocityProvider desiredVelocityProvider)
   {
      this.desiredVelocityProvider = desiredVelocityProvider;
   }

   /**
    * Sets the protocol for stop walking requests to the controller.
    *
    * @param stopWalkingMessenger the callback used to send requests.
    */
   public void setStopWalkingMessenger(StopWalkingMessenger stopWalkingMessenger)
   {
      this.stopWalkingMessenger = stopWalkingMessenger;
   }

   /**
    * Sets the protocol for start walking requests to the controller.
    *
    * @param startWalkingMessenger the callback used to send requests.
    */
   public void setStartWalkingMessenger(StartWalkingMessenger startWalkingMessenger)
   {
      this.startWalkingMessenger = startWalkingMessenger;
   }

   /**
    * Sets the protocol for sending desired velocities to the controller.
    *
    * @param directionalControlMessenger the callback used to send the directional control command.
    */
   public void setDirectionalControlMessenger(DirectionalControlMessenger directionalControlMessenger)
   {
      this.directionalControlMessenger = directionalControlMessenger;
   }
}
