package us.ihmc.simulationConstructionSetTools.tools;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class CIToolsTest
{
   private static final boolean SHOW_GUI = true;

   @Test
   public void testGetClassAndMethodName()
   {
      String classAndMethodName = CITools.getClassAndMethodName();
      assertEquals("CIToolsTest.testGetClassAndMethodName", classAndMethodName);

      classAndMethodName = CITools.getClassAndMethodName(0);
      assertEquals("CIToolsTest.testGetClassAndMethodName", classAndMethodName);
   }

   @Tag("gui-slow")
   @Test
   public void testLogMessages()
   {
      CITools.reportTestStartedMessage(SHOW_GUI);

      CITools.reportOutMessage("OutMessage1", SHOW_GUI);
      CITools.reportOutMessage("OutMessage2", SHOW_GUI);
      CITools.reportErrorMessage("ErrorMessage1", SHOW_GUI);
      CITools.reportErrorMessage("ErrorMessage2", SHOW_GUI);
      CITools.reportParameterMessage("ParameterMessage1", SHOW_GUI);
      CITools.reportParameterMessage("ParameterMessage2", SHOW_GUI);

      CITools.reportTestFinishedMessage(SHOW_GUI);
   }
}
