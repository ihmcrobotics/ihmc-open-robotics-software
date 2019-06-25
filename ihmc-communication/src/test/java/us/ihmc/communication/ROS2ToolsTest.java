package us.ihmc.communication;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import com.google.common.base.CaseFormat;
import std_msgs.msg.dds.Int64;
import std_msgs.msg.dds.Int8;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

class ROS2ToolsTest
{
   public static void main(String[] args)
   {
      new ROS2ToolsTest().testROS2Communication();
   }

   public void testROS2Communication()
   {
      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, getClass().getSimpleName());

      ROS2ModuleIdentifier id = new ROS2ModuleIdentifier("ihmc_test", "/test");

      String robotName = "skippy";
      IHMCROS2Publisher intPublisher = new IHMCROS2Publisher(ros2Node, Int64.class, null, null, null);

      new ROS2Callback<>(ros2Node, Int64.class, null, null, null, this::acceptMessage);

      new ExceptionHandlingThreadScheduler(getClass().getSimpleName()).schedule(() ->
      {
         Int64 num = new Int64();
         num.setData(System.nanoTime());
         LogTools.info("Publishing: {}", num.getData());
         intPublisher.publish(num);
      }, 1.0);

      try
      {
         Thread.currentThread().join();
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }

   private void acceptMessage(Int64 message)
   {
      LogTools.info("Received int: {}", message);
   }

   @Test
   void testToROSTopicFormat()
   {
      compareAgainstGuava("", "");
      compareAgainstGuava("a", "a");
      compareAgainstGuava("a", "A");
      compareAgainstGuava("arm_trajectory", "ArmTrajectory");
      compareAgainstGuava("behavior_control_mode", "BehaviorControlMode");
      compareAgainstGuava("there_is_a_pine_tree", "ThereIsAPineTree");
      compareAgainstGuava("you_are_dah_best", "youAreDahBest");
      compareAgainstGuava("v2_forge", "V2Forge");
      compareAgainstGuava("v578_forge", "V578Forge");
      compareAgainstGuava("forge_v578", "ForgeV578");
      compareAgainstGuava("forge578", "Forge578");
      compareAgainstGuava("578_forge", "578Forge");

      String troublingString = "REAIsAnAcronymAndIsNotHandledByGuava";
      String guavaExpectedAnswer = "r_e_a_is_an_acronym_and_is_not_handled_by_guava";
      String rosTopicExpectedName = "rea_is_an_acronym_and_is_not_handled_by_guava";
      assertEquals(guavaExpectedAnswer, CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, troublingString));
      assertEquals(rosTopicExpectedName, ROS2Tools.toROSTopicFormat(troublingString));

      troublingString = "TheAcronymIsAtTheEndREA";
      guavaExpectedAnswer = "the_acronym_is_at_the_end_r_e_a";
      rosTopicExpectedName = "the_acronym_is_at_the_end_rea";
      assertEquals(guavaExpectedAnswer, CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, troublingString));
      assertEquals(rosTopicExpectedName, ROS2Tools.toROSTopicFormat(troublingString));
   }

   private static void compareAgainstGuava(String expectedOutput, String stringToEvaluate)
   {
      String guavaOutput = CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, stringToEvaluate);
      String ros2ToolsOutput = ROS2Tools.toROSTopicFormat(stringToEvaluate);
      assertEquals(guavaOutput, ros2ToolsOutput);
      assertEquals(expectedOutput, ros2ToolsOutput);
   }
}
