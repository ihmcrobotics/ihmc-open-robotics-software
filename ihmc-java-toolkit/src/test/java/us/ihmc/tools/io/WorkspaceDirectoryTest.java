package us.ihmc.tools.io;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;

import java.nio.file.Path;

public class WorkspaceDirectoryTest
{
   @Test
   public void testWorkspaceDirectory()
   {
      WorkspaceDirectory workspaceDirectory = new WorkspaceDirectory("ihmc-open-robotics-software");
      LogTools.info("File access available: {}", workspaceDirectory.isFileAccessAvailable() ? "Yes" : "No");

      if (workspaceDirectory.isFileAccessAvailable())
      {
         Path directoryPath = workspaceDirectory.getDirectoryPath();
         LogTools.info("Directory path: {}", directoryPath);
         Assertions.assertNotNull(directoryPath, "Directory path is null");
      }
   }
}
