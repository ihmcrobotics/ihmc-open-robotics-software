package us.ihmc.tools.processManagement;

import static org.junit.Assert.fail;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.io.IOUtils;
import org.apache.commons.lang3.SystemUtils;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class ProcessSpawnerTest
{
   private static final Path testFilePath = Paths.get(System.getProperty("java.io.tmpdir"), "ProcessSpawnerTest.tmp");

   private void validateFileContents(String expectedContent) throws Exception
   {
      byte[] binaryData = new byte[128];
      DataInputStream dis = FileTools.newFileDataInputStream(testFilePath);
      dis.readFully(binaryData, 0, dis.available());
      dis.close();
      String content = new String(binaryData).trim();
      Assert.assertEquals(expectedContent, content);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 30000)
   public void testForkedShellProcessSpawner() throws Exception
   {
      String randomString = Long.toString(System.nanoTime());
      String[] arguments = {randomString};
      System.out.println(testFilePath);
      ForkedShellProcessSpawner sps = new ForkedShellProcessSpawner(true);
      sps.spawn("echo", arguments, testFilePath.toFile(), testFilePath.toFile(), null);

      while (sps.hasRunningProcesses())
      {
         ThreadTools.sleep(500);
      }

      sps.shutdown();
      validateFileContents(randomString);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 30000)
   public void testShelloutProcessSpawnerOnShellScript() throws Exception
   {
      if (SystemUtils.IS_OS_WINDOWS)
      {
         PrintTools.error("Not compatible with Windows");
         fail("Not compatible with Windows");

         return;
      }

      Path testScriptPath = extractScriptFileToKnownDirectory();
      String randomString = Long.toString(System.nanoTime());
      String[] arguments = {randomString};
      ShellScriptProcessSpawner sps = new ShellScriptProcessSpawner(true);
      sps.spawn(testScriptPath.toAbsolutePath().toString(), arguments, testFilePath.toFile(), testFilePath.toFile(), null);

      while (sps.hasRunningProcesses())
      {
         ThreadTools.sleep(500);
      }

      sps.shutdown();
      validateFileContents(randomString);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 30000)
   public void testJavaProcessSpawnerWithClasspathAsArgument() throws Exception
   {
      String randomString = Long.toString(System.nanoTime());
      String[] arguments = {randomString};
      JavaProcessSpawner jps = new JavaProcessSpawner(true, false);
      jps.spawn(getClass(), arguments);

      while (jps.hasRunningProcesses())
      {
         ThreadTools.sleep(500);
      }

      validateFileContents(randomString);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 30000)
   public void testJavaProcessSpawnerWithClasspathAsEnvironmentVariable() throws Exception
   {
      String randomString = Long.toString(System.nanoTime());
      String[] arguments = {randomString};
      JavaProcessSpawner jps = new JavaProcessSpawner(true, true);
      jps.spawn(getClass(), arguments);

      while (jps.hasRunningProcesses())
      {
         ThreadTools.sleep(500);
      }

      validateFileContents(randomString);
   }

   @ContinuousIntegrationTest(estimatedDuration = 2.1, categoriesOverride = IntegrationCategory.FLAKY)
   @Test(timeout = 30000)
   public void testExitListeners() throws Exception
   {
      if (SystemUtils.IS_OS_WINDOWS)
      {
         PrintTools.error("Not compatible with Windows");

         return;
      }

      final List<Integer> exitValues = new ArrayList<>(2);
      exitValues.clear();
      String[] arguments = {"2"};

      // implicit exit listener
      ForkedShellProcessSpawner sps = new ForkedShellProcessSpawner(true);
      sps.spawn("sleep", arguments, null, null, new ExitListener()
      {
         @Override
         public void exited(int statusValue)
         {
            exitValues.add(statusValue);
         }
      });

      // explicit exit listener
      Process p = sps.spawn("sleep", arguments);
      sps.setProcessExitListener(p, new ExitListener()
      {
         @Override
         public void exited(int statusValue)
         {
            exitValues.add(statusValue);
         }
      });

      while (sps.hasRunningProcesses())
      {
         ThreadTools.sleep(500);
      }

      Assert.assertEquals(exitValues.size(), 2);
      Assert.assertEquals(exitValues.get(0).intValue(), 0);
      Assert.assertEquals(exitValues.get(1).intValue(), 0);
   }

   private static Path extractScriptFileToKnownDirectory()
   {
      Path testScriptPath = Paths.get(System.getProperty("user.home"), ".ihmc", "testScript.sh");
      try
      {
         Files.createDirectories(testScriptPath.getParent());
         Files.write(testScriptPath, IOUtils.toByteArray(ProcessSpawner.class.getResourceAsStream("testScript.sh")));
         Runtime.getRuntime().exec("chmod u+x " + testScriptPath.toAbsolutePath().toString());
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      return testScriptPath;
   }

   // this one is used for testing JavaProcessSpawner
   public static void main(String[] args) throws Exception
   {
      DataOutputStream dos = FileTools.newFileDataOutputStream(testFilePath, DefaultExceptionHandler.PRINT_STACKTRACE);
      dos.writeBytes(args[0]);
      dos.flush();
      dos.close();
   }
}
