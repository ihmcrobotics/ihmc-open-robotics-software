package us.ihmc.quadrupedRobotics.output;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.OutputProcessor;

import java.util.ArrayList;

public class OutputProcessorBuilder
{

   private class OutputProcessorBank implements OutputProcessor
   {
      private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      private final ArrayList<OutputProcessorComponent> outputProcessors;

      public OutputProcessorBank(ArrayList<OutputProcessorComponent> outputProcessors)
      {
         this.outputProcessors = outputProcessors;
      }

      @Override
      public void initialize()
      {
         for (int i = 0; i < outputProcessors.size(); i++)
         {
            outputProcessors.get(i).initialize();
         }
      }

      @Override
      public void update()
      {
         for (int i = 0; i < outputProcessors.size(); i++)
         {
            outputProcessors.get(i).update();
         }
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return null;
      }

      @Override
      public String getDescription()
      {
         return null;
      }
   }

   private final FullRobotModel fullRobotModel;
   private final ArrayList<OutputProcessorComponent> outputProcessors;
   private boolean outputProcessorHasBeenBuilt;

   public OutputProcessorBuilder(FullRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;
      this.outputProcessors = new ArrayList<>();
      this.outputProcessorHasBeenBuilt = false;
   }

   public void addComponent(OutputProcessorComponent outputProcessorComponent)
   {
      outputProcessorComponent.setFullRobotModel(fullRobotModel);
      outputProcessors.add(outputProcessorComponent);
   }

   public OutputProcessor build()
   {
      if (outputProcessorHasBeenBuilt)
      {
         throw new RuntimeException("build() can only be called once for an instance of OutputProcessorBuilder");
      }
      else
      {
         outputProcessorHasBeenBuilt = true;
      }
      return new OutputProcessorBank(outputProcessors);
   }
}
