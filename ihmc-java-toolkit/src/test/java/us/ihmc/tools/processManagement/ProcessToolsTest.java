package us.ihmc.tools.processManagement;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class ProcessToolsTest
{

	@Test
   public void testGetAllSystemProcesses()
   {
      ArrayList<String> processLines = ProcessTools.getAllSystemProcesses();
      
      for (String line : processLines)
         PrintTools.info(this, line);
   }
}
