package us.ihmc.avatar.heightMap;

import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.perception.heightMap.HeightMapInputData;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.sensorProcessing.heightMap.HeightMapFilterParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

public class HeightMapMessagerAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category Root = apiFactory.createRootCategory("HeightMapRoot");
   private static final MessagerAPIFactory.CategoryTheme HeightMap = apiFactory.createCategoryTheme("HeightMap");

   public static final MessagerAPIFactory.Topic<Boolean> Clear = topic("Clear");

   public static final MessagerAPIFactory.Topic<Double> MaxHeight = topic("MaxHeight");
   public static final MessagerAPIFactory.Topic<Double> xPosition = topic("xPosition");
   public static final MessagerAPIFactory.Topic<Double> yPosition = topic("yPosition");
   public static final MessagerAPIFactory.Topic<Double> zPosition = topic("zPosition");

   public static final MessagerAPIFactory.Topic<HeightMapInputData> PointCloudData = topic("PointCloudData");
   public static final MessagerAPIFactory.Topic<HeightMapMessage> HeightMapData = topic("HeightMapData");
   public static final MessagerAPIFactory.Topic<HeightMapParameters> parameters = topic("Parameters");
   public static final MessagerAPIFactory.Topic<HeightMapFilterParameters> filterParameters = topic("FilterParameters");
   public static final MessagerAPIFactory.Topic<Double> GridCenterX = topic("GridCenterX");
   public static final MessagerAPIFactory.Topic<Double> GridCenterY = topic("GridCenterY");
   public static final MessagerAPIFactory.Topic<Integer> PublishFrequency = topic("PublishRate");
   public static final MessagerAPIFactory.Topic<Boolean> Export = topic("Export");
   public static final MessagerAPIFactory.Topic<Boolean> Import = topic("Import");

   public static final MessagerAPIFactory.Topic<Boolean> EnableUpdates = topic("EnableUpdates");

   public static final MessagerAPIFactory.MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return Root.child(HeightMap).topic(apiFactory.createTypedTopicTheme(name));
   }
}
