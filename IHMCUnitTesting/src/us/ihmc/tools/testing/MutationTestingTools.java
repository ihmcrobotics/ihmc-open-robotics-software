package us.ihmc.tools.testing;

import java.awt.Desktop;
import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

import org.pitest.mutationtest.commandline.MutationCoverageReport;

public class MutationTestingTools
{
   public static void doPITMutationTestAndOpenResult(String targetTests, String targetClasses)
   {
      String reportDirectoryName = "pit-reports";
      MutationCoverageReport.main(new String[] {"--reportDir", reportDirectoryName,
                                                "--targetClasses", targetClasses,
                                                "--targetTests", targetTests,
                                                "--sourceDirs", "src,test",
                                                "--mutators",
                                                "ALL"
//                                                "RETURN_VALS,"
//                                              + "INLINE_CONSTS,"
//                                              + "MATH,"
//                                              + "VOID_METHOD_CALLS,"
//                                              + "NEGATE_CONDITIONALS,"
//                                              + "CONDITIONALS_BOUNDARY,"
//                                              + "INCREMENTS,"
//                                              + "REMOVE_INCREMENTS,"
//                                              + "NON_VOID_METHOD_CALLS,"
//                                              + "CONSTRUCTOR_CALLS,"
//                                              + "REMOVE_CONDITIONALS_EQ_IF,"
//                                              + "REMOVE_CONDITIONALS_EQ_ELSE,"
//                                              + "REMOVE_CONDITIONALS_ORD_IF,"
//                                              + "REMOVE_CONDITIONALS_ORD_ELSE,"
//                                              + "REMOVE_CONDITIONALS,"
//                                              + "EXPERIMENTAL_MEMBER_VARIABLE,"
//                                              + "EXPERIMENTAL_SWITCH,"
//                                              + "EXPERIMENTAL_ARGUMENT_PROPAGATION,"
//                                              + "REMOVE_SWITCH"
                                                });

      File reportDirectory = new File(reportDirectoryName);
      if (reportDirectory.isDirectory() && reportDirectory.exists())
      {
         String[] list = reportDirectory.list();
         String lastDirectoryName = list[list.length-1];

         System.out.println("Found last directory " + lastDirectoryName);

         File reportFile = new File(reportDirectory, lastDirectoryName + "/index.html");
         String absolutePath;
         try
         {
            absolutePath = reportFile.getCanonicalPath();

            absolutePath = absolutePath.replace("\\", "/");
            System.out.println("Opening " + "file://" + absolutePath);

            URI uri = new URI("file://" + absolutePath);
            Desktop.getDesktop().browse(uri);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         catch (URISyntaxException e)
         {
            e.printStackTrace();
         }
      }
   }

   public static String createClassSelectorStringFromTargetString(String targetTests)
   {
      return targetTests.substring(0, targetTests.lastIndexOf('.')) + "*";
   }
}

