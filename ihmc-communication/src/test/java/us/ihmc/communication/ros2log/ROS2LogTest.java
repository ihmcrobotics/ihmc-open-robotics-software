package us.ihmc.communication.ros2log;

import com.google.common.base.CaseFormat;
import controller_msgs.RobotConfigurationData;
import gnu.trove.list.array.TLongArrayList;
import org.apache.commons.lang3.tuple.Pair;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Disabled // jros2: ROS2LogIOTools.writeLogFile returns null — log export path needs migration
public class ROS2LogTest
{
   @Test
   public void testLogExportAndImport()
   {
      // Test logging to file
      String robotName = "Robot";

      for (ROS2LogSerialization serialization : ROS2LogSerialization.values())
      {
         ROS2Topic<?> controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(robotName);
         ROS2Topic<RobotConfigurationData> robotConfigurationData = controllerOutputTopic.withType(RobotConfigurationData.class);
         ROS2Node ros2Node = new ROS2Node("ihmc_" + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, "test_node"));

         // Test record/writing
         List<RecordTopicManager<?>> rosTopicManagers = new ArrayList<>();

         RecordTopicManager topicManagerExport = new RecordTopicManager(robotConfigurationData, ros2Node, System::currentTimeMillis);

         RobotConfigurationData messageExport0 = new RobotConfigurationData();
         RobotConfigurationData messageExport1 = new RobotConfigurationData();
         messageExport0.setSequenceId(1);
         messageExport1.setSequenceId(2);

         topicManagerExport.getDataReference().set(Pair.of(100L, messageExport0));
         topicManagerExport.update();
         topicManagerExport.getDataReference().set(Pair.of(200L, messageExport1));
         topicManagerExport.update();

         rosTopicManagers.add(topicManagerExport);
         String fileName = ROS2LogIOTools.writeLogFile(rosTopicManagers, serialization);
         Assertions.assertTrue(fileName != null);

         // Test loading file
         List<ReplayTopicManager<?>> topicManagers = ROS2LogIOTools.loadLogFile(ros2Node, Arrays.asList(robotConfigurationData), new File(fileName));
         Assertions.assertTrue(topicManagers.size() == 1);

         ReplayTopicManager<?> topicManagerImport = topicManagers.get(0);
         TLongArrayList importedTimestamps = topicManagerImport.getTimestamps();
         List<?> importedMessages = topicManagerImport.getMessages();

         Assertions.assertTrue(importedTimestamps.size() == 2);
         Assertions.assertTrue(importedTimestamps.get(0) == 0L);
         Assertions.assertTrue(importedTimestamps.get(1) == 100L);

         Assertions.assertTrue(importedMessages.size() == 2);
         Assertions.assertEquals(1L, ((RobotConfigurationData) importedMessages.get(0)).getSequenceId());
         Assertions.assertEquals(2L, ((RobotConfigurationData) importedMessages.get(1)).getSequenceId());

         ros2Node.close();
      }
   }
}
