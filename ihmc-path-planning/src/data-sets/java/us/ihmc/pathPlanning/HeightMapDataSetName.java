package us.ihmc.pathPlanning;

import controller_msgs.msg.dds.HeightMapMessage;
import controller_msgs.msg.dds.HeightMapMessagePubSubType;
import geometry_msgs.Pose;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.io.IOException;
import java.io.InputStream;

public enum HeightMapDataSetName
{
   Cinders(new Pose3D(0.72, 0.2, -0.13, 0.1, 0.0, 0.0), new Pose3D(2.95, 0.2, -0.13, 0.1, 0.0, 0.0)),
   Obstacles_1(new Pose3D(0.0, 0.0, -0.18, 0.2, 0.0, 0.0), new Pose3D(3.4, 1.0, -0.18, 0.2, 0.0, 0.0)),
   Obstacles_2(new Pose3D(0.0, 0.0, -0.17, 0.2, 0.0, 0.0), new Pose3D(3.4, 1.0, -0.17, 0.2, 0.0, 0.0)),
   Stairs_1(new Pose3D(1.6, -1.85, -0.915, 1.6, 0.0, 0.0), new Pose3D(3.5, 0.2, 0.15, 0.05, 0.0, 0.0)),
   Stairs_2(new Pose3D(0.0, 0.0, -0.16, 0.0, 0.0, 0.0), new Pose3D(3.6, 0.17, 0.86, 0.6, 0.0, 0.0)),
   Stepping_Stones_1(new Pose3D(0.5, 0.1, -0.12, 0.4, 0.0, 0.0), new Pose3D(3.8, -0.5, 0.03, -0.9, 0.0, 0.0)),
   Stepping_Stones_2(new Pose3D(0.5, 0.1, -0.12, 0.4, 0.0, 0.0), new Pose3D(3.8, -0.5, 0.03, -0.9, 0.0, 0.0)),
   Stepping_Stones_3(new Pose3D(0.56, 0.13, -0.12, 0.4, 0.0, 0.0), new Pose3D(3.8, -0.5, 0.03, -0.9, 0.0, 0.0)),
   Stepping_Stones_4(new Pose3D(0.56, 0.13, -0.12, 0.4, 0.0, 0.0), new Pose3D(3.8, -0.5, 0.03, -0.9, 0.0, 0.0));

   private static final String DATA_SET_DIRECTORY_PATH = "us/ihmc/pathPlanning/heightMapDataSets";

   private final HeightMapData heightMapData;
   private final HeightMapMessage message;
   private final Pose3D start, goal;

   HeightMapDataSetName(Pose3D start, Pose3D goal)
   {
      this.start = start;
      this.goal = goal;

      Class<DataSetIOTools> loadingClass = DataSetIOTools.class;
      String dataSetPlanarRegionsPath = DATA_SET_DIRECTORY_PATH + "/" + name().toLowerCase() + ".json";
      InputStream inputStream = loadingClass.getClassLoader().getResourceAsStream(dataSetPlanarRegionsPath);
      JSONSerializer<HeightMapMessage> serializer = new JSONSerializer<>(new HeightMapMessagePubSubType());

      try
      {
         message = serializer.deserialize(inputStream);
         heightMapData = new HeightMapData(message);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public HeightMapMessage getMessage()
   {
      return message;
   }

   public HeightMapData getHeightMapData()
   {
      return heightMapData;
   }

   public Pose3D getStart()
   {
      return start;
   }

   public Pose3D getGoal()
   {
      return goal;
   }
}
