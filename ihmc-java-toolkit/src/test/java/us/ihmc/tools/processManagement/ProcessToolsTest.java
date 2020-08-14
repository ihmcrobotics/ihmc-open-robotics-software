package us.ihmc.tools.processManagement;

import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

public class ProcessToolsTest
{
	@Test
   public void testGetAllSystemProcesses()
   {
      ArrayList<String> processLines = ProcessTools.getAllSystemProcesses();
      
      for (String line : processLines)
      {
         LogTools.info(line);
      }

      assertFalse(processLines.isEmpty());
   }
}
