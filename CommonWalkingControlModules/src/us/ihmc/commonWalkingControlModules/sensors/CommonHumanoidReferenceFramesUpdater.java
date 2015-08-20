package us.ihmc.commonWalkingControlModules.sensors;

import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.robotController.SensorProcessor;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class CommonHumanoidReferenceFramesUpdater implements SensorProcessor
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final CommonHumanoidReferenceFrames refrenceFrames;

   public CommonHumanoidReferenceFramesUpdater(CommonHumanoidReferenceFrames refrenceFrames)
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
