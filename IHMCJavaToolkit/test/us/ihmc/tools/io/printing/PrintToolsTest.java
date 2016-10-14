package us.ihmc.tools.io.printing;

import static org.junit.Assert.*;
import static us.ihmc.tools.continuousIntegration.IntegrationCategory.*;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;

import org.junit.Test;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.io.printing.PrintTools;

public class PrintToolsTest
{
	@ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = EXCLUDE)
   @Test(timeout = 30000)
   public void testPrintTools() throws Exception
   {
      ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
      
      PrintStream systemOut = System.out;
      
      System.setOut(new PrintStream(byteArrayOutputStream));
      
      PrintTools.info(this, "Test log tools!");
      
      System.out.flush();
      
      System.setOut(systemOut);
      
      System.out.println("ByteArrayOutputStream.toString(): " + byteArrayOutputStream.toString());
      
      assertTrue("PrintTools didn't work.", byteArrayOutputStream.toString().startsWith("[INFO] (PrintToolsTest.java:25): Test log tools!"));
   }
}
