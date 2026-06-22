package us.ihmc.communication.ros2log;

import us.ihmc.communication.HumanoidROS2Topic;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.common.base.CaseFormat;
import gnu.trove.list.array.TLongArrayList;
import org.apache.commons.lang3.tuple.Pair;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import toolbox_msgs.KinematicsToolboxOutputStatus;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;

import java.io.File;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

/**
 * Regression test for ROS 2 log JSON encoding of {@link KinematicsToolboxOutputStatus}.
 * Broken jros2 logs stored base64 CDR blobs instead of readable JSON strings (see now.json vs old.json).
 */
public class ROS2LogKinematicsToolboxOutputStatusTest
{
   private static final String LEGACY_TOPIC_KEY = "toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus";

   @Test
   public void testKinematicsToolboxOutputStatusLogUsesJsonStringsNotBase64() throws Exception
   {
      ROS2Topic<KinematicsToolboxOutputStatus> topic = HumanoidROS2Topic.IHMC_ROOT.withType(KinematicsToolboxOutputStatus.class);
      ROS2Node ros2Node = new ROS2Node("ihmc_" + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, "ros2_log_test"));

      KinematicsToolboxOutputStatus exported = createSampleStatus();

      RecordTopicManager<KinematicsToolboxOutputStatus> topicManager = new RecordTopicManager<>(topic, ros2Node, () -> 100L);
      topicManager.getDataReference().set(Pair.of(100L, exported));
      topicManager.update();

      List<RecordTopicManager<?>> topicManagers = new ArrayList<>();
      topicManagers.add(topicManager);

      String fileName = ROS2LogIOTools.writeLogFile(topicManagers, ROS2LogSerialization.JSON);
      Assertions.assertNotNull(fileName);

      ObjectMapper objectMapper = ROS2LogSerialization.JSON.createObjectMapper();
      JsonNode rootNode = objectMapper.readTree(new File(fileName));
      JsonNode topicNode = rootNode.get(LEGACY_TOPIC_KEY);
      Assertions.assertNotNull(topicNode, "Expected legacy topic key in log file");

      String serializedMessage = topicNode.get(ROS2LogIOTools.messageKey).get(0).asText();
      Assertions.assertTrue(serializedMessage.startsWith("{"), "Log message must be JSON text, not base64 CDR");
      Assertions.assertTrue(serializedMessage.contains("currentToolboxState") || serializedMessage.contains("current_toolbox_state"),
                            "Serialized message should contain toolbox state field as readable JSON");
      Assertions.assertFalse(serializedMessage.startsWith("AAE"), "Log message must not be base64 CDR like broken now.json");

      List<ReplayTopicManager<?>> replayManagers = ROS2LogIOTools.loadLogFile(ros2Node, List.of(topic), new File(fileName));
      Assertions.assertEquals(1, replayManagers.size());

      KinematicsToolboxOutputStatus imported = (KinematicsToolboxOutputStatus) replayManagers.get(0).getMessages().get(0);
      assertKinematicsToolboxOutputStatusEqual(exported, imported);

      ros2Node.close();
   }

   @Test
   public void testLoadLegacyKinematicsToolboxLogSnippet() throws Exception
   {
      ROS2Topic<KinematicsToolboxOutputStatus> topic = HumanoidROS2Topic.IHMC_ROOT.withType(KinematicsToolboxOutputStatus.class);
      ROS2Node ros2Node = new ROS2Node("ihmc_" + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, "ros2_log_legacy_test"));

      File legacyLog;
      try (InputStream inputStream = getClass().getResourceAsStream("/ros2log/kinematics_toolbox_legacy_log_snippet.json"))
      {
         Assertions.assertNotNull(inputStream, "Missing test resource kinematics_toolbox_legacy_log_snippet.json");
         legacyLog = File.createTempFile("kinematics_toolbox_legacy", ".json");
         legacyLog.deleteOnExit();
         java.nio.file.Files.writeString(legacyLog.toPath(), new String(inputStream.readAllBytes(), StandardCharsets.UTF_8));
      }

      List<ReplayTopicManager<?>> replayManagers = ROS2LogIOTools.loadLogFile(ros2Node, List.of(topic), legacyLog);
      Assertions.assertEquals(1, replayManagers.size());

      ReplayTopicManager<?> replayManager = replayManagers.get(0);
      TLongArrayList timestamps = replayManager.getTimestamps();
      List<?> messages = replayManager.getMessages();

      Assertions.assertEquals(1, timestamps.size());
      Assertions.assertEquals(0L, timestamps.get(0));
      Assertions.assertEquals(1, messages.size());

      KinematicsToolboxOutputStatus imported = (KinematicsToolboxOutputStatus) messages.get(0);
      Assertions.assertEquals(0L, imported.getSequenceId());
      Assertions.assertEquals(KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_RUNNING, imported.getCurrentToolboxState());
      Assertions.assertEquals(-1087810655, imported.getJointNameHash());
      Assertions.assertFalse(imported.getDesiredJointAngles().isEmpty());
      Assertions.assertEquals(0.018464804f, imported.getDesiredJointAngles().get(0), 1.0e-6f);

      ros2Node.close();
   }

   private static KinematicsToolboxOutputStatus createSampleStatus()
   {
      KinematicsToolboxOutputStatus status = new KinematicsToolboxOutputStatus();
      status.setSequenceId(42L);
      status.setCurrentToolboxState(KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_RUNNING);
      status.setJointNameHash(-1087810655);
      status.getDesiredJointAngles().add(0.018464804f);
      status.getDesiredJointAngles().add(-0.0010407863f);
      status.getDesiredJointAngles().add(-0.17352305f);
      status.setSolutionQuality(0.95);
      status.setLeftFootInContact(true);
      status.setRightFootInContact(false);
      return status;
   }

   private static void assertKinematicsToolboxOutputStatusEqual(KinematicsToolboxOutputStatus expected, KinematicsToolboxOutputStatus actual)
   {
      Assertions.assertEquals(expected.getSequenceId(), actual.getSequenceId());
      Assertions.assertEquals(expected.getCurrentToolboxState(), actual.getCurrentToolboxState());
      Assertions.assertEquals(expected.getJointNameHash(), actual.getJointNameHash());
      Assertions.assertEquals(expected.getDesiredJointAngles().size(), actual.getDesiredJointAngles().size());
      for (int i = 0; i < expected.getDesiredJointAngles().size(); i++)
      {
         Assertions.assertEquals(expected.getDesiredJointAngles().get(i), actual.getDesiredJointAngles().get(i), 1.0e-6f);
      }
      Assertions.assertEquals(expected.getSolutionQuality(), actual.getSolutionQuality(), 1.0e-9);
      Assertions.assertEquals(expected.getLeftFootInContact(), actual.getLeftFootInContact());
      Assertions.assertEquals(expected.getRightFootInContact(), actual.getRightFootInContact());
   }
}
