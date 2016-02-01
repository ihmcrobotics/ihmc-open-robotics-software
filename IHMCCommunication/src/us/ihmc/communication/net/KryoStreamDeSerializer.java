package us.ihmc.communication.net;

import java.io.IOException;
import java.io.InputStream;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.io.Input;

/**
 * Serialize objects, write length and object to stream and read objects from stream
 * 
 * Not thread safe
 * 
 * @author Jesper Smith
 *
 */
public class KryoStreamDeSerializer
{

   private final Kryo kryo;

   private final byte[] readBuffer;
   private final Input input;
   
   
   public KryoStreamDeSerializer(int inputBufferSize)
   {
      kryo = new Kryo();
      kryo.setReferences(false);
      kryo.setRegistrationRequired(true);
      
      readBuffer = new byte[inputBufferSize];

      input = new Input();
      
      input.setBuffer(readBuffer);
   }
   
   /**
    * Register class with Kryo te serialize
    * 
    * @param clazz
    */
   public void registerClass(Class<?> clazz)
   {
      kryo.register(clazz);
   }
   
   /**
    * Register classes using a netclasslist
    */
   public void registerClasses(NetClassList netClassList)
   {
      for(Class<?> clazz : netClassList.getPacketClassList())
      {
         kryo.register(clazz);
      }
      
      for(Class<?> clazz : netClassList.getPacketFieldList())
      {
         kryo.register(clazz);
      }
      
   }

   /**
    * Blocking read for the next object
    * 
    * @param inputStream to read from
    * @return next object
    * @throws IOException
    */
   public Object read(InputStream inputStream) throws IOException
   {
      int length = readInt(inputStream);
      final int readLength = inputStream.read(readBuffer, 0, length);
      if(readLength != length)
      {
         throw new IOException("Cannot read whole object, expected " + length + " bytes, got " + readLength + " bytes");
      }
      input.setBuffer(readBuffer, 0, length);
      return kryo.readClassAndObject(input);
   }

   private int readInt(InputStream inputStream) throws IOException
   {
      final int readLength = inputStream.read(readBuffer, 0, 4);
      if (readLength != 4)
      {
         throw new IOException("Cannot read length of object");
      }
      int length = (readBuffer[0] & 0xFF) << 24 | (readBuffer[1] & 0xFF) << 16 | (readBuffer[2] & 0xFF) << 8 | (readBuffer[3] & 0xFF);

      return length;
   }

}
