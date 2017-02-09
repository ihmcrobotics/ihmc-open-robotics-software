package us.ihmc.simulationconstructionset.bambooTools;

import static org.junit.Assert.assertEquals;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;

public class BambooToolsTest
{
   private static final boolean SHOW_GUI = true;
   
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetClassAndMethodName()
   {
      String classAndMethodName = BambooTools.getClassAndMethodName();
      assertEquals("BambooToolsTest.testGetClassAndMethodName", classAndMethodName);
      
      classAndMethodName = BambooTools.getClassAndMethodName(0);
      assertEquals("BambooToolsTest.testGetClassAndMethodName", classAndMethodName);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = { IntegrationCategory.UI })
	@Test(timeout=300000)
   public void testLogMessagesToFile() throws IOException
   {
      BambooTools.reportTestStartedMessage(SHOW_GUI);
      
      BambooTools.reportOutMessage("OutMessage1", SHOW_GUI);
      BambooTools.reportOutMessage("OutMessage2", SHOW_GUI);
      BambooTools.reportErrorMessage("ErrorMessage1", SHOW_GUI);
      BambooTools.reportErrorMessage("ErrorMessage2", SHOW_GUI);
      BambooTools.reportParameterMessage("ParameterMessage1", SHOW_GUI);
      BambooTools.reportParameterMessage("ParameterMessage2", SHOW_GUI);
      
      BambooTools.reportTestFinishedMessage(SHOW_GUI);
      
      Path path = Paths.get("testResources/us/ihmc/simulationconstructionset/bambooTools/testMessages.txt");
      BambooTools.logMessagesToFile(path.toFile());

      FileInputStream fileInputStream = new FileInputStream(path.toString());
      BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(fileInputStream));

      String readLine = bufferedReader.readLine();
      while((readLine = bufferedReader.readLine()) != null)
      {
         System.out.println(readLine);
      }
      
      bufferedReader.close();
   }
}
