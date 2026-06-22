package us.ihmc.communication.ros2log;

import us.ihmc.communication.HumanoidROS2Topic;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import toolbox_msgs.KinematicsToolboxOutputStatus;
import us.ihmc.jros2.ROS2Topic;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.Consumer;

/**
 * Regression test for replaying legacy JSON logs that serialize Euclid wrappers as strings:
 * {@code {"point":"(x, y, z)"}} and {@code {"quaternion":"(x, y, z, w)"}}.
 */
public class ROS2LogReplayLegacyEuclidFieldsTest
{
   private static final String LEGACY_TOPIC_KEY = "toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus";
   private static final String STATUS_PAYLOAD_KEY = "toolbox_msgs::msg::dds_::KinematicsToolboxOutputStatus_";

   @Test
   public void testReplayParsesLegacyStringifiedEuclidFields()
   {
      File logFile = createLegacyStringifiedEuclidLog();
      ROS2Topic<KinematicsToolboxOutputStatus> topic = HumanoidROS2Topic.IHMC_ROOT.withType(KinematicsToolboxOutputStatus.class);

      AtomicLong replayClock = new AtomicLong(0L);
      List<KinematicsToolboxOutputStatus> replayedStatuses = new ArrayList<>();

      ROS2LogReplay replay = new ROS2LogReplay(List.of(topic),
                                               logFile,
                                               ignoredTopic -> (Consumer<Object>) message -> replayedStatuses.add((KinematicsToolboxOutputStatus) message),
                                               replayClock::get);

      try
      {
         Assertions.assertTrue(replay.isReady(), "Replay should load the test log");

         // First call initializes replay timing (now = 0), second call advances so timestamp 0 is published.
         replay.doIncrementalReplay();
         replayClock.set(1L);
         replay.doIncrementalReplay();

         Assertions.assertFalse(replayedStatuses.isEmpty(), "Expected one replayed status message");
         KinematicsToolboxOutputStatus latestStatus = replayedStatuses.get(replayedStatuses.size() - 1);

         Assertions.assertEquals(-0.002, latestStatus.getDesiredRootPosition().getPoint().getX(), 1.0e-9);
         Assertions.assertEquals(-0.001, latestStatus.getDesiredRootPosition().getPoint().getY(), 1.0e-9);
         Assertions.assertEquals(0.901, latestStatus.getDesiredRootPosition().getPoint().getZ(), 1.0e-9);

         Assertions.assertEquals(-0.001, latestStatus.getDesiredRootOrientation().getQuaternion().getX(), 1.0e-9);
         Assertions.assertEquals(0.002, latestStatus.getDesiredRootOrientation().getQuaternion().getY(), 1.0e-9);
         Assertions.assertEquals(0.0, latestStatus.getDesiredRootOrientation().getQuaternion().getZ(), 1.0e-9);
         Assertions.assertEquals(1.0, latestStatus.getDesiredRootOrientation().getQuaternion().getS(), 1.0e-9);

         Assertions.assertEquals(-0.002, latestStatus.getDesiredTorsoPosition().getPoint().getX(), 1.0e-9);
         Assertions.assertEquals(-0.001, latestStatus.getDesiredTorsoPosition().getPoint().getY(), 1.0e-9);
         Assertions.assertEquals(0.919, latestStatus.getDesiredTorsoPosition().getPoint().getZ(), 1.0e-9);

         Assertions.assertEquals(-0.001, latestStatus.getDesiredTorsoOrientation().getQuaternion().getX(), 1.0e-9);
         Assertions.assertEquals(0.002, latestStatus.getDesiredTorsoOrientation().getQuaternion().getY(), 1.0e-9);
         Assertions.assertEquals(0.0, latestStatus.getDesiredTorsoOrientation().getQuaternion().getZ(), 1.0e-9);
         Assertions.assertEquals(1.0, latestStatus.getDesiredTorsoOrientation().getQuaternion().getS(), 1.0e-9);
      }
      finally
      {
         replay.destroy();
      }
   }

   private static File createLegacyStringifiedEuclidLog()
   {
      try
      {
         ObjectMapper objectMapper = ROS2LogSerialization.JSON.createObjectMapper();
         ObjectNode rootNode = objectMapper.createObjectNode();
         ObjectNode topicNode = rootNode.putObject(LEGACY_TOPIC_KEY);
         topicNode.putArray(ROS2LogIOTools.timestampKey).add(0L);

         ObjectNode statusNode = objectMapper.createObjectNode();
         statusNode.put("sequenceId", 0L);
         statusNode.put("jointNameHash", 2006668507);
         statusNode.put("currentToolboxState", KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_RUNNING);
         statusNode.set("desiredRootPosition", createStringifiedWrapper(objectMapper, "point", "(-0.002, -0.001,  0.901 )"));
         statusNode.set("desiredRootOrientation", createStringifiedWrapper(objectMapper, "quaternion", "(-0.001,  0.002,  0.000,  1.000 )"));
         statusNode.set("desiredTorsoPosition", createStringifiedWrapper(objectMapper, "point", "(-0.002, -0.001,  0.919 )"));
         statusNode.set("desiredTorsoOrientation", createStringifiedWrapper(objectMapper, "quaternion", "(-0.001,  0.002, -0.000,  1.000 )"));

         ObjectNode payloadNode = objectMapper.createObjectNode();
         payloadNode.set(STATUS_PAYLOAD_KEY, statusNode);

         ArrayNode messages = topicNode.putArray(ROS2LogIOTools.messageKey);
         messages.add(objectMapper.writeValueAsString(payloadNode));

         File logFile = File.createTempFile("ros2_log_legacy_stringified_euclid", ".json");
         logFile.deleteOnExit();
         objectMapper.writerWithDefaultPrettyPrinter().writeValue(logFile, rootNode);
         return logFile;
      }
      catch (Exception e)
      {
         throw new RuntimeException("Failed to create legacy replay log fixture", e);
      }
   }

   private static ObjectNode createStringifiedWrapper(ObjectMapper objectMapper, String fieldName, String value)
   {
      ObjectNode wrapper = objectMapper.createObjectNode();
      wrapper.put(fieldName, value);
      return wrapper;
   }
}
