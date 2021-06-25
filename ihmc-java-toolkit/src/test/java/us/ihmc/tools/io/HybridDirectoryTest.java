package us.ihmc.tools.io;

import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;

import java.nio.file.Path;
import java.nio.file.Paths;

public class HybridDirectoryTest
{
   @Test
   public void testConstructors()
   {
      Path dotIHMC = Paths.get(System.getProperty("user.home")).resolve(".ihmc");
      String openRobotics = "ihmc-open-robotics-software";
      String subsequentPathToResourceFolder = "ihmc-java-toolkit/src/test/resources";
      HybridDirectory directory = new HybridDirectory(dotIHMC, openRobotics, subsequentPathToResourceFolder, "/");
      LogTools.info(directory.getWorkspaceDirectory());
      LogTools.info(directory.getExternalDirectory());
      LogTools.info(directory.getClassForLoading());
//      LogTools.info(directory.getClasspathResource());

      directory = new HybridDirectory(dotIHMC, openRobotics, subsequentPathToResourceFolder, getClass());
      LogTools.info(directory.getWorkspaceDirectory());
      LogTools.info(directory.getExternalDirectory());
      LogTools.info(directory.getClassForLoading());

      directory = new HybridDirectory(dotIHMC, openRobotics, subsequentPathToResourceFolder, getClass(), "/root");
      LogTools.info(directory.getWorkspaceDirectory());
      LogTools.info(directory.getExternalDirectory());
      LogTools.info(directory.getClassForLoading());

      directory = new HybridDirectory(dotIHMC, openRobotics, subsequentPathToResourceFolder, getClass(), "subsequent");
      LogTools.info(directory.getWorkspaceDirectory());
      LogTools.info(directory.getExternalDirectory());
      LogTools.info(directory.getClassForLoading());
   }
}
