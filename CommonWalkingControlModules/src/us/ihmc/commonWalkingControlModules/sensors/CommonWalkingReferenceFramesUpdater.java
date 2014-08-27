package us.ihmc.commonWalkingControlModules.sensors;

import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import com.yobotics.simulationconstructionset.robotController.SensorProcessor;

public class CommonWalkingReferenceFramesUpdater implements SensorProcessor
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final CommonWalkingReferenceFrames refrenceFrames;

   public CommonWalkingReferenceFramesUpdater(CommonWalkingReferenceFrames refrenceFrames)
   {
      this.refrenceFrames = refrenceFrames;
   }

   public void initialize()
   {
      update();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   public void update()
   {
      refrenceFrames.updateFrames();
   }
}
