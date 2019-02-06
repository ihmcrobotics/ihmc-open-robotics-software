package us.ihmc.tools.factories;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.Assertions;
import us.ihmc.commons.RunnableThatThrows;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class FactoryToolsTest
{
   @Test
   public void testFactoryDisposes()
   {
      final ExampleValidFactory exampleFactory = new ExampleValidFactory();
      exampleFactory.setRequiredField1(1.0);
      exampleFactory.setRequiredField2(1.0);
      exampleFactory.createObject();
      
      Assertions.assertExceptionThrown(FactoryDisposedException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            exampleFactory.createObject();
         }
      });
   }
   
   @Test
   public void testFactoryRequiresFieldsSet()
   {
      final ExampleValidFactory exampleFactory = new ExampleValidFactory();
      exampleFactory.setRequiredField1(1.0);
      
      Assertions.assertExceptionThrown(FactoryFieldNotSetException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            exampleFactory.createObject();
         }
      });
   }
   
   @Test
   public void testFactoryRequiresOptionalNotNull()
   {
      final ExampleInvalidFactory exampleFactory = new ExampleInvalidFactory();
      exampleFactory.setRequiredField1(1.0);
      exampleFactory.setRequiredField2(1.0);
      
      Assertions.assertExceptionThrown(NullPointerException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            exampleFactory.createObject();
         }
      });
   }
}
