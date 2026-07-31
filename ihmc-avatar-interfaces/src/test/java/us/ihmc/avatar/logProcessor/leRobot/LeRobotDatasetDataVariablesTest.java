package us.ihmc.avatar.logProcessor.leRobot;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.nio.file.Path;
import java.util.Collections;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class LeRobotDatasetDataVariablesTest
{
   @TempDir
   Path temporaryDirectory;

   @Test
   void testAlexKSTTargetAndFingerOrder()
   {
      assertEquals(28, LeRobotDatasetDataVariables.STATE_SIZE);
      assertEquals(28, LeRobotDatasetDataVariables.ACTION_SIZE);

      List<String> actionFeatureNames = LeRobotDatasetDataVariables.getActionFeatureNames();
      assertEquals(LeRobotDatasetDataVariables.ACTION_SIZE, actionFeatureNames.size());
      assertEquals("left_gripper_x", actionFeatureNames.get(0));
      assertEquals("left_gripper_qs", actionFeatureNames.get(6));
      assertEquals("right_gripper_x", actionFeatureNames.get(7));
      assertEquals("right_gripper_qs", actionFeatureNames.get(13));
      assertEquals("neck_y", actionFeatureNames.get(14));
      assertEquals("neck_z", actionFeatureNames.get(15));
      assertEquals("left_ability_hand_index_q1", actionFeatureNames.get(16));
      assertEquals("left_ability_hand_thumb_q2", actionFeatureNames.get(21));
      assertEquals("right_ability_hand_index_q1", actionFeatureNames.get(22));
      assertEquals("right_ability_hand_thumb_q2", actionFeatureNames.get(27));

      List<String> stateFeatureNames = LeRobotDatasetDataVariables.getStateFeatureNames();
      assertEquals(LeRobotDatasetDataVariables.STATE_SIZE, stateFeatureNames.size());
      assertEquals(actionFeatureNames, stateFeatureNames);
   }

   @Test
   void writesTheFullRecordedSchemaWithoutChangingDevelopPolicyDefaults() throws Exception
   {
      LeRobotDataset dataset = new LeRobotDataset(temporaryDirectory.resolve("dataset"), null, null);
      dataset.mkdirs();
      dataset.writeMetaJson();

      JsonNode info = new ObjectMapper().readTree(temporaryDirectory.resolve("dataset/meta/info.json").toFile());
      assertEquals(28, LeRobotDataset.RECORDED_STATE_SIZE);
      assertEquals(28, info.at("/features/observation.state/shape/0").asInt());
      assertEquals(28, info.at("/features/action/shape/0").asInt());
      assertEquals("v3.0", info.get("codebase_version").asText());
   }

   @Test
   void recomputesNonVisualStatisticsForLoadedRecords()
   {
      LeRobotDatasetEpisode episode = new LeRobotDatasetEpisode(null, 0, "test task", null, null);
      List<Float> zeros = Collections.nCopies(28, 0.0f);
      List<Float> ones = Collections.nCopies(28, 1.0f);
      episode.getRecords().add(new LeRobotEpisodeRecord(zeros, ones, 0, 0, 0.0f, 1, "log", false, 0, 0));
      episode.getRecords().add(new LeRobotEpisodeRecord(ones, zeros, 0, 1, 0.1f, 2, "log", true, 1, 0));

      episode.recomputeStatistics();
      JsonNode stats = new ObjectMapper().createObjectNode();
      episode.getStatistics().writeJson((com.fasterxml.jackson.databind.node.ObjectNode) stats, new SideDependentList<>());

      assertEquals(28, stats.get("observation.state").get("mean").size());
      assertEquals(0.5, stats.get("observation.state").get("mean").get(0).asDouble());
      assertEquals(2, stats.get("action").get("count").get(0).asInt());
   }

   @Test
   void refusesToFinalizeAnEmptyDataset()
   {
      LeRobotDataset dataset = new LeRobotDataset(temporaryDirectory.resolve("empty"), null, null);
      dataset.mkdirs();
      assertThrows(IllegalStateException.class, dataset::finalizeDataset);
   }
}
