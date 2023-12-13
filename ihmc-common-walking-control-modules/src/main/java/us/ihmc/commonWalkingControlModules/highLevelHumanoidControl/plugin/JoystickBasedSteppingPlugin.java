package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.*;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.List;

public class JoystickBasedSteppingPlugin implements HumanoidSteppingPlugin
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoEnum<HighLevelControllerName> latestHighLevelControllerStatus = new YoEnum<>("LatestHighLevelControllerStatePlugin", registry, HighLevelControllerName.class);

   private final ComponentBasedFootstepDataMessageGenerator stepGenerator;
   private final VelocityBasedSteppingPlugin fastWalkingJoystickPlugin;

   private final List<Updatable> updatables;

   public JoystickBasedSteppingPlugin(ComponentBasedFootstepDataMessageGenerator stepGenerator,
                                      VelocityBasedSteppingPlugin fastWalkingStepGenerator,
                                      List<Updatable> updatables)
   {
      this.stepGenerator = stepGenerator;
      this.fastWalkingJoystickPlugin = fastWalkingStepGenerator;
      this.updatables = updatables;
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
      for (int i = 0; i < updatables.size(); i++)
         updatables.get(i).update(time);

      if (latestHighLevelControllerStatus.getValue() == HighLevelControllerName.CUSTOM1)
         fastWalkingJoystickPlugin.update(time);
      else if (latestHighLevelControllerStatus.getValue() == HighLevelControllerName.WALKING)
         stepGenerator.update(time);
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
      latestHighLevelControllerStatus.set(HighLevelControllerName.fromByte(statusMessage.getEndHighLevelControllerName()));
   }

   public void addUpdatable(Updatable updatable)
   {
      this.updatables.add(updatable);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return stepGenerator.getSCS2YoGraphics();
   }
}
