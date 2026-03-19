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

public class HumanoidSteppingPlugin implements HighLevelHumanoidControllerPlugin
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoEnum<HighLevelControllerName> latestHighLevelControllerStatus = new YoEnum<>("LatestHighLevelControllerStatePlugin", registry, HighLevelControllerName.class);

   private final ContinuousStepGenerator stepGenerator;

   private final List<Updatable> updatables;

   public HumanoidSteppingPlugin(ContinuousStepGenerator stepGenerator,
                                 List<Updatable> updatables)
   {
      this.stepGenerator = stepGenerator;
      this.updatables = updatables;
      registry.addChild(stepGenerator.getRegistry());
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

      if (latestHighLevelControllerStatus.getValue() == HighLevelControllerName.WALKING)
         stepGenerator.update(time);
   }

   public void setFootstepAdjustment(FootstepAdjustment footstepAdjustment)
   {
      stepGenerator.setFootstepAdjustment(footstepAdjustment);
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
