package us.ihmc.tools.processManagement;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.processManagement.ProcessTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class ProcessToolsTest
{

	@DeployableTestMethod(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testGetAllSystemProcesses()
   {
      ArrayList<String> processLines = ProcessTools.getAllSystemProcesses();
      
      for (String line : processLines)
         PrintTools.info(this, line);
   }
}
