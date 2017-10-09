package us.ihmc.communication.net;

import java.io.IOException;
import java.io.OutputStream;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.io.Output;

/**
 * Serialize objects, write length and object to stream and read objects from stream
 * 
 * Not thread safe
 * 
 * @author Jesper Smith
 *
 */
public class KryoStreamSerializer
{

   private final Kryo kryo;

   private final byte[] writeBuffer;

   private final Output output;
   
   
   public KryoStreamSerializer(int outputBufferSize)
   {
      kryo = new Kryo();
      kryo.setReferences(false);
      kryo.setRegistrationRequired(true);
      
      writeBuffer = new byte[outputBufferSize];

      output = new Output();
      
      output.setBuffer(writeBuffer);
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
    * Blocking write to an outputStream
    * 
    * @param outputStream output stream to write to
    * @param object object to write
    * 
    * @return Number of bytes written
    * 
    * @throws IOException
    */
   public int write(OutputStream outputStream, Object object) throws IOException
   {
      output.clear();
      output.setPosition(4);
      kryo.writeClassAndObject(output, object);
      int length = output.position() - 4;
      writeIntToWriteBuffer(length);
      outputStream.write(writeBuffer, 0, output.position());
      return length + 4;
   }

   private void writeIntToWriteBuffer(int value) throws IOException
   {
      
      writeBuffer[0] = ((byte) (value >> 24));
      writeBuffer[1] = ((byte) (value >> 16));
      writeBuffer[2] = ((byte) (value >> 8));
      writeBuffer[3] = ((byte) (value));
      
   }

}
