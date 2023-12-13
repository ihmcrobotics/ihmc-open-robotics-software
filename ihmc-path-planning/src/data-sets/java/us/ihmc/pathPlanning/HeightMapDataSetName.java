package us.ihmc.pathPlanning;

import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.HeightMapMessagePubSubType;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;

import java.io.*;

public enum HeightMapDataSetName
{
   Cinders_1(new Pose3D(0.72, 0.2, -0.13, 0.1, 0.0, 0.0), new Pose3D(2.95, 0.2, -0.13, 0.1, 0.0, 0.0)),
   Cinders_2(new Pose3D(0.2, 0.2, -0.16, 0.1, 0.0, 0.0), new Pose3D(4.0, -1.2, -0.16, -0.1, 0.0, 0.0)),
   Obstacles_1(new Pose3D(0.0, 0.0, -0.18, 0.2, 0.0, 0.0), new Pose3D(3.4, 1.0, -0.18, 0.2, 0.0, 0.0)),
   Obstacles_2(new Pose3D(0.0, 0.0, -0.17, 0.2, 0.0, 0.0), new Pose3D(3.4, 1.0, -0.17, 0.2, 0.0, 0.0)),
   Ramp(new Pose3D(0.3, 0.0, -0.18, 0.0, 0.0, 0.0), new Pose3D(2.85, -0.4, 0.28, 0.15, 0.0, 0.0)),
   Ramp_2(new Pose3D(-0.3, -0.8, -0.16, -1.8, 0.0, 0.0), new Pose3D(-2.85, -0.33, 0.29, 1.5, 0.0, 0.0)),
   Ramp_3(new Pose3D(-0.3, -0.03, -0.03, -2.0, 0.0, 0.0), new Pose3D(-2.65, 0.42, 0.43, 2.1, 0.0, 0.0)),
   Stairs_1(new Pose3D(2.2, -1.85, -0.915, 1.6, 0.0, 0.0), new Pose3D(3.5, 0.2, 0.15, 0.05, 0.0, 0.0)),
   Stairs_2(new Pose3D(0.0, 0.0, -0.16, 0.0, 0.0, 0.0), new Pose3D(3.6, 0.17, 0.86, 0.6, 0.0, 0.0)),
   Stepping_Stones_1(new Pose3D(0.5, 0.1, -0.12, 0.4, 0.0, 0.0), new Pose3D(3.8, -0.5, 0.03, -0.9, 0.0, 0.0)),
   Stepping_Stones_2(new Pose3D(0.5, 0.1, -0.12, 0.4, 0.0, 0.0), new Pose3D(3.8, -0.5, 0.03, -0.9, 0.0, 0.0)),
   Stepping_Stones_3(new Pose3D(0.56, 0.13, -0.12, 0.4, 0.0, 0.0), new Pose3D(3.8, -0.5, 0.03, -0.9, 0.0, 0.0)),
//   Stepping_Stones_4(new Pose3D(0.56, 0.13, -0.12, 0.4, 0.0, 0.0), new Pose3D(3.8, -0.5, 0.03, -0.9, 0.0, 0.0)),
   Stepping_Stones_5(new Pose3D(0.2, -0.2, -0.3, 0.3, 0.0, 0.0), new Pose3D(2.5, -0.7, -0.1, -0.9, 0.0, 0.0)),
   Obstacle_Course(new Pose3D(1.4, 0.43, -0.15, 1.5, 0.0, 0.0), new Pose3D(5.34, 3.84, 0.39, 0.8, 0.0, 0.0));

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
         heightMapData = HeightMapMessageTools.unpackMessage(message);
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
