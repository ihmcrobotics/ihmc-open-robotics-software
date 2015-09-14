package us.ihmc.tools.io.printing;

import static org.junit.Assert.*;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;

import org.junit.Test;

import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class PrintToolsTest
{
	@DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLogTools() throws Exception
   {
      ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
      
      PrintStream systemOut = System.out;
      
      System.setOut(new PrintStream(byteArrayOutputStream));
      
      PrintTools.info(this, "Test log tools!");
      
      System.out.flush();
      
      System.setOut(systemOut);
      
      System.out.println("ByteArrayOutputStream.toString(): " + byteArrayOutputStream.toString());
      
      assertTrue("PrintTools didn't work.", byteArrayOutputStream.toString().startsWith("[INFO] (PrintToolsTest.java:24): Test log tools!"));
   }
}
