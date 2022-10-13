package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.*;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JoystickBasedSteppingPlugin implements HighLevelHumanoidControllerPlugin
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final MutableObject<HighLevelControllerName> latestHighLevelControllerStatus = new MutableObject<>(null);

   private final ComponentBasedFootstepDataMessageGenerator stepGenerator;
   private final VelocityBasedSteppingGenerator fastWalkingJoystickPlugin;

   public JoystickBasedSteppingPlugin(ComponentBasedFootstepDataMessageGenerator stepGenerator, VelocityBasedSteppingGenerator fastWalkingStepGenerator)
   {
      this.stepGenerator = stepGenerator;
      this.fastWalkingJoystickPlugin = fastWalkingStepGenerator;
      registry.addChild(stepGenerator.getRegistry());
      registry.addChild(fastWalkingStepGenerator.getRegistry());
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }

   @Override
   public void update(double time)
   {
      if (latestHighLevelControllerStatus.getValue() != HighLevelControllerName.CUSTOM1 || latestHighLevelControllerStatus.getValue() != HighLevelControllerName.WALKING)
         return;

      stepGenerator.update(time);
      fastWalkingJoystickPlugin.update(time);
   }

   public void setFootstepAdjustment(FootstepAdjustment footstepAdjustment)
   {
      stepGenerator.setFootstepAdjustment(footstepAdjustment);
//      fastWalkingJoystickPlugin.setFo
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
