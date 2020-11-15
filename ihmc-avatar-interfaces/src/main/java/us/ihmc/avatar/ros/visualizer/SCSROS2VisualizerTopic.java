package us.ihmc.avatar.ros.visualizer;

import java.util.HashMap;

public class SCSROS2VisualizerTopic
{
   private final String name;
   private final String partition;

//   private final HashMap<String, > types = new HashMap<>();

   public SCSROS2VisualizerTopic(String name, String partition)
   {
      this.name = name;
      this.partition = partition;
   }
}
