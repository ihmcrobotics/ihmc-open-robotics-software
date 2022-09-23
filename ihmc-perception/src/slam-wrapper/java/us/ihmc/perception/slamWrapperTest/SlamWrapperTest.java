package us.ihmc.perception.slamWrapperTest;

import org.bytedeco.javacpp.Loader;
import org.bytedeco.slamWrapper.SlamWrapper;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class SlamWrapperTest
{
   static
   {
      System.setProperty("org.bytedeco.javacpp.loadlibraries", "false");
   }

   public static void main(String[] args)
   {
      List<String> libraryFiles = new ArrayList<>();
      libraryFiles.add("libmetis-gtsam.so");
      libraryFiles.add("libgtsam.so");
      libraryFiles.add("libgtsam.so.4");
      libraryFiles.add("libgtsam.so.4.2.0");
      libraryFiles.add("libgtsam_unstable.so");
      libraryFiles.add("libgtsam_unstable.so.4");
      libraryFiles.add("libgtsam_unstable.so.4.2.0");
      libraryFiles.add("libjniSlamWrapper.so");
      libraryFiles.add("libslam-wrapper.so");

      for (String libraryFile : libraryFiles) {
         try
         {
            Loader.cacheResource(libraryFile);
            File cacheDir = Loader.getCacheDir();
            System.load(cacheDir.getAbsolutePath() + "/main/" + libraryFile);
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }
      }

      SlamWrapper.FactorGraphExternal factorGraphExternal = new SlamWrapper.FactorGraphExternal();

      factorGraphExternal.helloWorldTest();
   }
}
