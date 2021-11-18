package us.ihmc.avatar.heightMap;

import controller_msgs.msg.dds.HeightMapMessage;
import org.apache.commons.lang3.tuple.Pair;
import sensor_msgs.PointCloud2;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

class HeightMapMessagerAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category Root = apiFactory.createRootCategory("HeightMapRoot");
   private static final MessagerAPIFactory.CategoryTheme HeightMap = apiFactory.createCategoryTheme("HeightMap");

   public static final MessagerAPIFactory.Topic<Pair<PointCloud2, FramePose3D>> PointCloudData = topic("PointCloudData");
   public static final MessagerAPIFactory.Topic<HeightMapMessage> HeightMapData = topic("HeightMapData");
   public static final MessagerAPIFactory.Topic<HeightMapParameters> parameters = topic("Parameters");
   public static final MessagerAPIFactory.Topic<Point2D> GridCenter = topic("GridCenter");
   public static final MessagerAPIFactory.Topic<Integer> PublishFrequency = topic("PublishRate");
   public static final MessagerAPIFactory.Topic<Boolean> Export = topic("Export");

   public static final MessagerAPIFactory.MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return Root.child(HeightMap).topic(apiFactory.createTypedTopicTheme(name));
   }
}
