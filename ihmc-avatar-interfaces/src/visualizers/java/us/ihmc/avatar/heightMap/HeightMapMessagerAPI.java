package us.ihmc.avatar.heightMap;

import controller_msgs.msg.dds.HeightMapMessage;
import sensor_msgs.PointCloud2;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

class HeightMapMessagerAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category Root = apiFactory.createRootCategory("HeightMapRoot");
   private static final MessagerAPIFactory.CategoryTheme HeightMap = apiFactory.createCategoryTheme("HeightMap");

   // Perception data
   public static final MessagerAPIFactory.Topic<PointCloud2> PointCloudData = topic("PointCloudData");
   public static final MessagerAPIFactory.Topic<HeightMapMessage> HeightMapData = topic("HeightMapData");

   // Visualization control

   // Parameters
   public static final MessagerAPIFactory.Topic<HeightMapParameters> parameters = topic("Parameters");

   public static final MessagerAPIFactory.MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return Root.child(HeightMap).topic(apiFactory.createTypedTopicTheme(name));
   }
}
