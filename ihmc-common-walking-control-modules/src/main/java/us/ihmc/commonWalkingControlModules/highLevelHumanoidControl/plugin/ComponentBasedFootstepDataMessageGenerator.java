package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ComponentBasedFootstepDataMessageGenerator implements HighLevelHumanoidControllerPlugin
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

   public ContinuousStepGenerator getContinuousStepGenerator()
   {
      return continuousStepGenerator;
   }
}
