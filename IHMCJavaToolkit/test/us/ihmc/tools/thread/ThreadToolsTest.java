package us.ihmc.tools.thread;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;

import org.apache.commons.lang3.SystemUtils;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;

public class ThreadToolsTest
{
   /**
    * Tests giving stringed commands to runCommandLine
    * does not produce an error, which is the most likely
    * failure mode.
    */
   @EstimatedDuration(duration = 0.1)
   @Test(timeout = 30000)
   public void testRunCommandLineStringedCommmands()
   {
      if (SystemUtils.IS_OS_WINDOWS)
      {
         ThreadTools.runCommandLine("dir & cd .. & dir");
      }
      else if (SystemUtils.IS_OS_MAC)
      {
         ThreadTools.runCommandLine("ls -a; cd .");
      }
      else if (SystemUtils.IS_OS_LINUX)
      {
         ThreadTools.runCommandLine("ls -a; cd .");
      }
      else
      {
         Assert.fail("Using unsupported OS");
      }
   }

   /**
    * Tests capturing the output of an echo.
    */
   @EstimatedDuration(duration = 0.1)
   @Test(timeout = 30000)
   public void testRunCommandLineEchoOutput()
   {
      String bambooJobName = System.getProperty("BAMBOO_JOB_NAME");
      if (bambooJobName != null)
      {
         System.out.println("Not compatable with Bamboo. Job name: " + bambooJobName);
         return;
      }

      final StringBuilder commandLineOutput = new StringBuilder();

      PrintStream commandOutput = new PrintStream(new OutputStream()
      {
         @Override
         public void write(int b) throws IOException
         {
            commandLineOutput.append((char) b);
         }
      });

      ThreadTools.runCommandLine("echo Hi", commandOutput);

      Assert.assertTrue("Output not correct: " + commandLineOutput.toString(), commandLineOutput.toString().matches("Hi\\s*"));
   }
}
