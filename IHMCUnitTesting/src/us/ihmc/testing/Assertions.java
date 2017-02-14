package us.ihmc.testing;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;

import us.ihmc.tools.thread.RunnableThatThrows;

public class Assertions
{
   public static void assertExceptionThrown(Class<? extends Throwable> exceptionType, RunnableThatThrows methodToRun)
   {
      boolean thrown = false;
      
      try
      {
         methodToRun.run();
      }
      catch (Throwable throwable)
      {
         assertTrue("Exception type mismatch: Expected: " + exceptionType.getName() + " Actual: " + throwable.getClass().getName(), exceptionType.getName().equals(throwable.getClass().getName()));
         
         thrown = true;
      }
      
      assertTrue("Exception not thrown", thrown);
   }

   public static void assertSerializable(Serializable serializable)
   {
      assertSerializable(serializable, false);
   }

   public static void assertSerializable(Serializable serializable, boolean testEquals)
   {
      try
      {
         ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
         ObjectOutputStream objectOutputStream = new ObjectOutputStream(byteArrayOutputStream);
         objectOutputStream.writeObject(serializable);

         ByteArrayInputStream byteArrayInputStream = new ByteArrayInputStream(byteArrayOutputStream.toByteArray());
         ObjectInputStream objectInputStream = new ObjectInputStream(byteArrayInputStream);
         Object object = objectInputStream.readObject();
         if (testEquals)
         {
            assertTrue(object.equals(serializable));
         }
      }
      catch (IOException e)
      {
         fail("Object not serializable");
      }
      catch (ClassNotFoundException ce)
      {
         fail("Object not serializable: class not found");
      }
   }
}
