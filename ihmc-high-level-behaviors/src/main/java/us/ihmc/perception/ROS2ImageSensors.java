package us.ihmc.perception;

import std_msgs.Empty;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2DemandGraphTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.realsense.RealSenseImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class ROS2ImageSensors
{
   public static final Map<ROS2Topic<? extends ROS2Message<?>>, Integer> STEPPING_REALSENSE_TOPIC_MAP = Map.of(PerceptionAPI.STEPPING_REALSENSE_COLOR,
                                                                                                          RealSenseImageSensor.COLOR_IMAGE_KEY,
                                                                                                          PerceptionAPI.STEPPING_REALSENSE_DEPTH,
                                                                                                          RealSenseImageSensor.DEPTH_IMAGE_KEY);

   public static final Map<ROS2Topic<? extends ROS2Message<?>>, Integer> EXPERIMENTAL_ZED_TOPIC_MAP = Map.of(PerceptionAPI.EXPERIMENTAL_ZED_COLOR.get(RobotSide.LEFT),
                                                                                                        ZEDImageSensor.LEFT_COLOR_IMAGE_KEY,
                                                                                                        PerceptionAPI.EXPERIMENTAL_ZED_COLOR.get(RobotSide.RIGHT),
                                                                                                        ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY,
                                                                                                        PerceptionAPI.EXPERIMENTAL_ZED_DEPTH,
                                                                                                        ZEDImageSensor.DEPTH_IMAGE_KEY);

   public static final Map<ROS2Topic<? extends ROS2Message<?>>, Integer> STEPPING_ZED_TOPIC_MAP = Map.of(PerceptionAPI.STEPPING_ZED_COLOR.get(RobotSide.LEFT),
                                                                                                        ZEDImageSensor.LEFT_COLOR_IMAGE_KEY,
                                                                                                        PerceptionAPI.STEPPING_ZED_COLOR.get(RobotSide.RIGHT),
                                                                                                        ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY,
                                                                                                        PerceptionAPI.STEPPING_ZED_DEPTH,
                                                                                                        ZEDImageSensor.DEPTH_IMAGE_KEY);

   private final ROS2Node ros2Node;

   private final Map<String, ImageSensor> imageSensors = new HashMap<>();
   private final Map<String, ROS2DemandGraphNode> sensorDemandNodes = new HashMap<>();
   private final Map<String, Set<ImageSensorPublishThread>> publishThreads = new HashMap<>();
   private final Map<String, ROS2DemandGraphNode> publishDemandNodes = new HashMap<>();

   public ROS2ImageSensors(ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
   }

   public void addImageSensor(String sensorId, ImageSensor imageSensor, ReferenceFrame sensorFrame, ROS2Topic<Empty> demandTopic)
   {
      imageSensor.setSensorFrame(sensorFrame);
      imageSensors.put(sensorId, imageSensor);

      if (demandTopic == null)
      {
         imageSensor.run(true);
      }
      else
      {
         ROS2DemandGraphNode sensorDemandNode = new ROS2DemandGraphNode(ros2Node, demandTopic);
         sensorDemandNode.addDemandChangedCallback(imageSensor::run);
         sensorDemandNodes.put(sensorId, sensorDemandNode);
      }
   }

   public void publishSensor(String sensorId, Map<ROS2Topic<? extends ROS2Message<?>>, Integer> topicMap, ROS2Topic<Empty> demandTopic)
   {
      publishSensor(sensorId, topicMap, demandTopic, ros2Node);
   }

   public void publishSensor(String sensorId, Map<ROS2Topic<? extends ROS2Message<?>>, Integer> topicMap, ROS2Topic<Empty> demandTopic, ROS2Node ros2Node)
   {
      ImageSensor sensor = imageSensors.get(sensorId);
      ImageSensorPublishThread publishThread = new ImageSensorPublishThread(ros2Node, sensor);

      for (Map.Entry<ROS2Topic<? extends ROS2Message<?>>, Integer> entry : topicMap.entrySet())
         publishThread.addTopic(entry.getKey(), entry.getValue());

      publishThreads.putIfAbsent(sensorId, new HashSet<>());
      publishThreads.get(sensorId).add(publishThread);

      if (demandTopic == null)
      {
         publishThread.startRepeating();
      }
      else
      {
         ROS2DemandGraphNode publishDemandNode = new ROS2DemandGraphNode(ros2Node, demandTopic);
         publishDemandNodes.put(sensorId, publishDemandNode);

         ROS2DemandGraphNode sensorDemandNode = sensorDemandNodes.get(sensorId);
         if (sensorDemandNode != null)
            sensorDemandNode.addDependents(publishDemandNode);

         ROS2DemandGraphTools.runWhileDemanded(publishThread, publishDemandNode);
      }
   }

   public ImageSensor getSensor(String sensorId)
   {
      return imageSensors.get(sensorId);
   }

   public void destroy()
   {
      for (ImageSensor sensor : imageSensors.values())
         sensor.close();

      for (ROS2DemandGraphNode node : sensorDemandNodes.values())
         node.destroy();

      for (Set<ImageSensorPublishThread> threads : publishThreads.values())
         for (ImageSensorPublishThread thread : threads)
            thread.kill();

      for (ROS2DemandGraphNode node : publishDemandNodes.values())
         node.destroy();
   }
}