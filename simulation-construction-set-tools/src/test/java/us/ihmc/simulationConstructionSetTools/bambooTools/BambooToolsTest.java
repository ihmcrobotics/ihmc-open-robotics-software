package us.ihmc.simulationConstructionSetTools.bambooTools;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;

public class BambooToolsTest
{
   private static final boolean SHOW_GUI = true;

   @Test
   public void testGetClassAndMethodName()
   {
      String classAndMethodName = BambooTools.getClassAndMethodName();
      assertEquals("BambooToolsTest.testGetClassAndMethodName", classAndMethodName);

      classAndMethodName = BambooTools.getClassAndMethodName(0);
      assertEquals("BambooToolsTest.testGetClassAndMethodName", classAndMethodName);
   }

   @Tag("gui-slow")
   @Test
   public void testLogMessages()
   {
      BambooTools.reportTestStartedMessage(SHOW_GUI);

      BambooTools.reportOutMessage("OutMessage1", SHOW_GUI);
      BambooTools.reportOutMessage("OutMessage2", SHOW_GUI);
      BambooTools.reportErrorMessage("ErrorMessage1", SHOW_GUI);
      BambooTools.reportErrorMessage("ErrorMessage2", SHOW_GUI);
      BambooTools.reportParameterMessage("ParameterMessage1", SHOW_GUI);
      BambooTools.reportParameterMessage("ParameterMessage2", SHOW_GUI);

      BambooTools.reportTestFinishedMessage(SHOW_GUI);
   }
}
