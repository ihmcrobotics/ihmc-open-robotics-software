package us.ihmc.communication.ros2log;

import us.ihmc.communication.HumanoidROS2Topic;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.common.base.CaseFormat;
import ihmc_common_msgs.Capsule3DMessage;
import ihmc_common_msgs.SO3TrajectoryPointMessage;
import org.apache.commons.lang3.tuple.Pair;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * Regression test for ROS 2 log JSON round-trip of Euclid geometry wrapper messages.
 */
public class ROS2LogEuclidGeometryMessageTest
{
   @Test
   public void testCapsule3DMessageEuclidPointAndVectorRoundTrip() throws Exception
   {
      ROS2Topic<Capsule3DMessage> topic = HumanoidROS2Topic.IHMC_ROOT.withType(Capsule3DMessage.class);
      ROS2Node ros2Node = new ROS2Node("ihmc_" + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, "ros2_log_euclid_capsule_test"));

      Capsule3DMessage exported = createSampleCapsule3DMessage();

      RecordTopicManager<Capsule3DMessage> topicManager = new RecordTopicManager<>(topic, ros2Node, () -> 100L);
      topicManager.getDataReference().set(Pair.of(100L, exported));
      topicManager.update();

      List<RecordTopicManager<?>> topicManagers = new ArrayList<>();
      topicManagers.add(topicManager);

      String fileName = ROS2LogIOTools.writeLogFile(topicManagers, ROS2LogSerialization.JSON);
      Assertions.assertNotNull(fileName);

      ObjectMapper objectMapper = ROS2LogSerialization.JSON.createObjectMapper();
      JsonNode rootNode = objectMapper.readTree(new File(fileName));
      String topicKey = ROS2LogMessageCodec.topicKeyForMessageClass(Capsule3DMessage.class);
      JsonNode topicNode = rootNode.get(topicKey);
      Assertions.assertNotNull(topicNode, "Expected topic key in log file");

      String serializedMessage = topicNode.get(ROS2LogIOTools.messageKey).get(0).asText();
      Assertions.assertTrue(serializedMessage.contains("\"x\""), "Position should serialize as flat x,y,z JSON");
      Assertions.assertFalse(serializedMessage.contains("Point3D {"), "Position must not serialize as Point3D toString");

      List<ReplayTopicManager<?>> replayManagers = ROS2LogIOTools.loadLogFile(ros2Node, List.of(topic), new File(fileName));
      Assertions.assertEquals(1, replayManagers.size());

      Capsule3DMessage imported = (Capsule3DMessage) replayManagers.get(0).getMessages().get(0);
      assertCapsule3DMessageEqual(exported, imported);

      ros2Node.close();
   }

   @Test
   public void testSO3TrajectoryPointMessageEuclidQuaternionRoundTrip() throws Exception
   {
      ROS2Topic<SO3TrajectoryPointMessage> topic = HumanoidROS2Topic.IHMC_ROOT.withType(SO3TrajectoryPointMessage.class);
      ROS2Node ros2Node = new ROS2Node("ihmc_" + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, "ros2_log_euclid_quaternion_test"));

      SO3TrajectoryPointMessage exported = createSampleSO3TrajectoryPointMessage();

      RecordTopicManager<SO3TrajectoryPointMessage> topicManager = new RecordTopicManager<>(topic, ros2Node, () -> 100L);
      topicManager.getDataReference().set(Pair.of(100L, exported));
      topicManager.update();

      List<RecordTopicManager<?>> topicManagers = new ArrayList<>();
      topicManagers.add(topicManager);

      String fileName = ROS2LogIOTools.writeLogFile(topicManagers, ROS2LogSerialization.JSON);
      Assertions.assertNotNull(fileName);

      ObjectMapper objectMapper = ROS2LogSerialization.JSON.createObjectMapper();
      JsonNode rootNode = objectMapper.readTree(new File(fileName));
      String topicKey = ROS2LogMessageCodec.topicKeyForMessageClass(SO3TrajectoryPointMessage.class);
      JsonNode topicNode = rootNode.get(topicKey);
      Assertions.assertNotNull(topicNode, "Expected topic key in log file");

      String serializedMessage = topicNode.get(ROS2LogIOTools.messageKey).get(0).asText();
      Assertions.assertTrue(serializedMessage.contains("\"w\""), "Quaternion should serialize with w component");
      Assertions.assertFalse(serializedMessage.contains("Quaternion {"), "Orientation must not serialize as Quaternion toString");

      List<ReplayTopicManager<?>> replayManagers = ROS2LogIOTools.loadLogFile(ros2Node, List.of(topic), new File(fileName));
      Assertions.assertEquals(1, replayManagers.size());

      SO3TrajectoryPointMessage imported = (SO3TrajectoryPointMessage) replayManagers.get(0).getMessages().get(0);
      assertSO3TrajectoryPointMessageEqual(exported, imported);

      ros2Node.close();
   }

   private static Capsule3DMessage createSampleCapsule3DMessage()
   {
      Capsule3DMessage message = new Capsule3DMessage();
      message.getPosition().getPoint().set(1.25, -0.5, 2.75);
      message.getAxis().getVector().set(0.0, 0.0, 1.0);
      message.setRadius(0.15);
      message.setLength(0.8);
      return message;
   }

   private static SO3TrajectoryPointMessage createSampleSO3TrajectoryPointMessage()
   {
      SO3TrajectoryPointMessage message = new SO3TrajectoryPointMessage();
      message.setSequenceId(7L);
      message.setTime(1.5);
      message.getOrientation().getQuaternion().set(0.1, 0.2, 0.3, 0.9);
      message.getAngularVelocity().getVector().set(0.01, -0.02, 0.03);
      return message;
   }

   private static void assertCapsule3DMessageEqual(Capsule3DMessage expected, Capsule3DMessage actual)
   {
      Assertions.assertEquals(expected.getPosition().getPoint().getX(), actual.getPosition().getPoint().getX(), 1.0e-9);
      Assertions.assertEquals(expected.getPosition().getPoint().getY(), actual.getPosition().getPoint().getY(), 1.0e-9);
      Assertions.assertEquals(expected.getPosition().getPoint().getZ(), actual.getPosition().getPoint().getZ(), 1.0e-9);
      Assertions.assertEquals(expected.getAxis().getVector().getX(), actual.getAxis().getVector().getX(), 1.0e-9);
      Assertions.assertEquals(expected.getAxis().getVector().getY(), actual.getAxis().getVector().getY(), 1.0e-9);
      Assertions.assertEquals(expected.getAxis().getVector().getZ(), actual.getAxis().getVector().getZ(), 1.0e-9);
      Assertions.assertEquals(expected.getRadius(), actual.getRadius(), 1.0e-9);
      Assertions.assertEquals(expected.getLength(), actual.getLength(), 1.0e-9);
   }

   private static void assertSO3TrajectoryPointMessageEqual(SO3TrajectoryPointMessage expected, SO3TrajectoryPointMessage actual)
   {
      Assertions.assertEquals(expected.getSequenceId(), actual.getSequenceId());
      Assertions.assertEquals(expected.getTime(), actual.getTime(), 1.0e-9);
      Assertions.assertEquals(expected.getOrientation().getQuaternion().getX(), actual.getOrientation().getQuaternion().getX(), 1.0e-9);
      Assertions.assertEquals(expected.getOrientation().getQuaternion().getY(), actual.getOrientation().getQuaternion().getY(), 1.0e-9);
      Assertions.assertEquals(expected.getOrientation().getQuaternion().getZ(), actual.getOrientation().getQuaternion().getZ(), 1.0e-9);
      Assertions.assertEquals(expected.getOrientation().getQuaternion().getS(), actual.getOrientation().getQuaternion().getS(), 1.0e-9);
      Assertions.assertEquals(expected.getAngularVelocity().getVector().getX(), actual.getAngularVelocity().getVector().getX(), 1.0e-9);
      Assertions.assertEquals(expected.getAngularVelocity().getVector().getY(), actual.getAngularVelocity().getVector().getY(), 1.0e-9);
      Assertions.assertEquals(expected.getAngularVelocity().getVector().getZ(), actual.getAngularVelocity().getVector().getZ(), 1.0e-9);
   }
}
