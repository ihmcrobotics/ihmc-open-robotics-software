package us.ihmc.tools.gui;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.tools.gui.GUIMessagePanel;
import us.ihmc.tools.testing.TestPlanAnnotations;
import us.ihmc.tools.testing.TestPlanAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.testing.TestPlanAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.TestPlanTarget;

@ContinuousIntegrationPlan(targets = {TestPlanTarget.UI})
public class GUIMessagePanelTest
{
	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout = 30000)
   public void testGetText()
   {
      GUIMessagePanel guiMessagePanel = new GUIMessagePanel("Test");
      
      guiMessagePanel.appendMessage("message0");
      guiMessagePanel.appendMessage("message1");
      
      String text = guiMessagePanel.getText();
            
      assertEquals("message1\nmessage0\n", text);
   }
}
