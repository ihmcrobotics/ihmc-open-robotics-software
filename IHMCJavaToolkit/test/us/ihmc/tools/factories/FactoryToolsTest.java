package us.ihmc.tools.factories;

import org.junit.Test;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.thread.RunnableThatThrows;

public class FactoryToolsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testFactoryDisposes()
   {
      final ExampleValidFactory exampleFactory = new ExampleValidFactory();
      exampleFactory.setRequiredField1(1.0);
      exampleFactory.setRequiredField2(1.0);
      exampleFactory.createObject();
      
      JUnitTools.assertExceptionThrown(FactoryDisposedException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            exampleFactory.createObject();
         }
      });
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testFactoryRequiresFieldsSet()
   {
      final ExampleValidFactory exampleFactory = new ExampleValidFactory();
      exampleFactory.setRequiredField1(1.0);
      
      JUnitTools.assertExceptionThrown(FactoryFieldNotSetException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            exampleFactory.createObject();
         }
      });
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testFactoryRequiresOptionalNotNull()
   {
      final ExampleInvalidFactory exampleFactory = new ExampleInvalidFactory();
      exampleFactory.setRequiredField1(1.0);
      exampleFactory.setRequiredField2(1.0);
      
      JUnitTools.assertExceptionThrown(NullPointerException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            exampleFactory.createObject();
         }
      });
   }
}
