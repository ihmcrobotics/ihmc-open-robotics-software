package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.DesiredTurningVelocityProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.DesiredVelocityProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.DirectionalControlMessenger;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

public class VelocityBasedSteppingGenerator implements SteppingPlugin
{
   private final static Vector2DReadOnly zero2D = new Vector2D();

   private DesiredVelocityProvider desiredVelocityProvider = () -> zero2D;
   private DesiredTurningVelocityProvider desiredTurningVelocityProvider = () -> 0.0;
   private DirectionalControlMessenger directionalControlMessenger;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final String variableNameSuffix = "FWJ";

   private final YoBoolean ignoreWalkInputProvider = new YoBoolean("ignoreWalkInputProvider" + variableNameSuffix, registry);
   private final YoBoolean walk = new YoBoolean("walk" + variableNameSuffix, registry);
   private final YoBoolean walkPreviousValue = new YoBoolean("walkPreviousValue" + variableNameSuffix, registry);
   private final YoVelocityBasedSteppingParameters inputParameters = new YoVelocityBasedSteppingParameters("FSG", registry);

   private final YoInteger numberOfTicksBeforeSubmittingCommands = new YoInteger("numberOfTicksBeforeSubmittingFootsteps" + variableNameSuffix, registry);

   private BooleanProvider walkInputProvider;

   private final MutableObject<HighLevelControllerName> latestHighLevelControllerStatus = new MutableObject<>(null);

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
      if (!ignoreWalkInputProvider.getBooleanValue() && walkInputProvider != null)
         walk.set(walkInputProvider.getValue());

      if (latestHighLevelControllerStatus.getValue() != HighLevelControllerName.CUSTOM1)
         walk.set(false);

      if (!walk.getValue())
      {
         walkPreviousValue.set(false);
         return;
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

      if (walk.getValue() && directionalControlMessenger != null)
      {
         if (counter >= numberOfTicksBeforeSubmittingCommands.getValue())
         {
            directionalControlMessenger.submitDirectionalControlRequest(desiredVelocityX, desiredVelocityY, turningVelocity);
            counter = 0;
         }
         else
         {
            counter++;
         }
      }
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
    * Sets the protocol for sending desired velocities to the controller.
    *
    * @param directionalControlMessenger the callback used to send the directional control command.
    */
   public void setDirectionalControlMessenger(DirectionalControlMessenger directionalControlMessenger)
   {
      this.directionalControlMessenger = directionalControlMessenger;
   }

   public void setHighLevelStateChangeStatusListener(StatusMessageOutputManager statusMessageOutputManager)
   {
      statusMessageOutputManager.attachStatusMessageListener(HighLevelStateChangeStatusMessage.class, this::consumeHighLevelStateChangeStatus);
   }

   public void consumeHighLevelStateChangeStatus(HighLevelStateChangeStatusMessage statusMessage)
   {
      latestHighLevelControllerStatus.setValue(HighLevelControllerName.fromByte(statusMessage.getEndHighLevelControllerName()));
   }
}
