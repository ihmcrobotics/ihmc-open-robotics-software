package us.ihmc.simulationconstructionset.bambooTools;

import static org.junit.Assert.assertEquals;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;

import org.junit.Test;

import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class BambooToolsTest
{

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testGetClassAndMethodName()
   {
      String classAndMethodName = BambooTools.getClassAndMethodName();
      assertEquals("BambooToolsTest.testGetClassAndMethodName", classAndMethodName);
      
      classAndMethodName = BambooTools.getClassAndMethodName(0);
      assertEquals("BambooToolsTest.testGetClassAndMethodName", classAndMethodName);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testLogMessagesToFile() throws IOException
   {
      BambooTools.reportTestStartedMessage();
      
      BambooTools.reportOutMessage("OutMessage1");
      BambooTools.reportOutMessage("OutMessage2");
      BambooTools.reportErrorMessage("ErrorMessage1");
      BambooTools.reportErrorMessage("ErrorMessage2");
      BambooTools.reportParameterMessage("ParameterMessage1");
      BambooTools.reportParameterMessage("ParameterMessage2");
      
      BambooTools.reportTestFinishedMessage();

      File file = new File("testMessages.txt");
      BambooTools.logMessagesToFile(file);

      FileInputStream fileInputStream = new FileInputStream(file);
      BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(fileInputStream));

      StringBuilder builder = new StringBuilder();
      
      while(true)
      {
         String readLine = bufferedReader.readLine();
         if (readLine == null) break;

         builder.append(readLine);
      }
      
      System.out.println(builder.toString());
      bufferedReader.close();
      
      file.delete();
   }

}
