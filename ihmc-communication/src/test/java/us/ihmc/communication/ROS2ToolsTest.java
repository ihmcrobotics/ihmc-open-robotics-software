package us.ihmc.communication;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import com.google.common.base.CaseFormat;

class ROS2ToolsTest
{

   @Test
   void testToROSTopicFormat()
   {
      compareAgainstGuava("");
      compareAgainstGuava("a");
      compareAgainstGuava("A");
      compareAgainstGuava("ArmTrajectory");
      compareAgainstGuava("BehaviorControlMode");
      compareAgainstGuava("ThereIsAPineTree");
      compareAgainstGuava("youAreDahBest");
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

   private static void compareAgainstGuava(String stringToEvaluate)
   {
      String expected =  CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, stringToEvaluate);
      String actual = ROS2Tools.toROSTopicFormat(stringToEvaluate);
      assertEquals(expected, actual);
   }
}
