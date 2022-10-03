package us.ihmc.promp.test;

import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.promp.global.promp.*;

public class EigenTest
{
   private static void loadLibraries() throws IOException
   {
      // We need to disable javacpp from trying to automatically load libraries.
      // Otherwise, it will try to load them by name when they aren't in the library path
      // (LD_LIBRARY_PATH on Linux).
      //
      // The approach taken here is to use System.load to load each library by explicit
      // absolute path on disk.
      System.setProperty("org.bytedeco.javacpp.loadlibraries", "false");

      List<String> libraryFiles = new ArrayList<>();
      libraryFiles.add("libpromp.so");
      libraryFiles.add("libjnipromp.so");

      WorkspaceDirectory resourcesDirectory = new WorkspaceDirectory("ihmc-open-robotics-software", "promp/src/main/resources");
      for (String libraryFile : libraryFiles)
      {
         System.load(new WorkspaceFile(resourcesDirectory, libraryFile).getFilePath().toAbsolutePath().normalize().toString());
      }
   }

   public static void main(String[] args)
   {
      try
      {
         loadLibraries();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      EigenMatrixXd matrixXd = new EigenMatrixXd(2, 2);

      matrixXd.apply(0, 0).put(3);
      matrixXd.apply(1, 0).put(2.5);
      matrixXd.apply(0, 1).put(-1);
      matrixXd.apply(1, 1).put(matrixXd.coeff(1, 0) + matrixXd.coeff(0, 1));

      matrixXd.debugPrint();
   }
}
