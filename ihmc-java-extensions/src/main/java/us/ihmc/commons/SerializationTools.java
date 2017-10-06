package us.ihmc.commons;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;

public class SerializationTools
{
   /**
    * Checks if an object is serializable by trying to write it
    * to an ObjectOutputStream and calling flush.
    * 
    * @param objectToTest object to check
    * @return If the object is serializable.
    */
   public static boolean isSerializable(Object objectToTest)
   {
      try (ObjectOutputStream testStream = new ObjectOutputStream(new ByteArrayOutputStream()))
      {
         testStream.writeObject(objectToTest);
         testStream.flush();
         return true;
      }
      catch (IOException ioException)
      {
         ioException.printStackTrace();
         return false;
      }
   }
}
