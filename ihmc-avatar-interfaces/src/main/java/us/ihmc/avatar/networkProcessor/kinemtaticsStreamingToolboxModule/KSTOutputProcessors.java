package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;

import java.util.ArrayList;
import java.util.List;

public class KSTOutputProcessors implements KSTOutputProcessor
{
   private final List<KSTOutputProcessor> outputProcessors = new ArrayList<>();

   public KSTOutputProcessors()
   {
   }

   public void add(KSTOutputProcessor outputProcessor)
   {
      outputProcessors.add(outputProcessor);
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
   public void update(double time, boolean wasStreaming, boolean isStreaming, KinematicsToolboxOutputStatus latestOutput)
   {
      for (int i = 0; i < outputProcessors.size(); i++)
      {
         outputProcessors.get(i).update(time, wasStreaming, isStreaming, latestOutput);
      }
   }

   @Override
   public KinematicsToolboxOutputStatus getProcessedOutput()
   {
      if (outputProcessors.isEmpty())
         return null;
      return outputProcessors.get(outputProcessors.size() - 1).getProcessedOutput();
   }
}
