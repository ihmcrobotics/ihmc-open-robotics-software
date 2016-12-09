package us.ihmc.multicastLogDataProtocol.control;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.SelectionKey;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.io.ByteBufferInputStream;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;

import us.ihmc.multicastLogDataProtocol.StreamingDataTCPClient;

// Kryo is only temporary, re-use handshake protobuf for generic improvements to protocol 
public class LogHandshake
{
   public static final byte STREAM_REQUEST = 0x7A;
   public static final byte HANDSHAKE_REQUEST = 0x6A;
   public static final short header = 0x6A6A;
   
   public byte[] protoShake;

   public String modelLoaderClass = null;
   public String modelName;
   public byte[] model;
   public String[] resourceDirectories;
   public byte[] resourceZip;
   
   public boolean createSummary;
   public String summaryTriggerVariable;
   public String[] summarizedVariables;
   
   private static Kryo getKryo()
   {
      Kryo kryo = new Kryo();
      kryo.register(LogHandshake.class);
      kryo.register(byte[].class);
      kryo.register(String.class);
      kryo.register(String[].class);
      kryo.setRegistrationRequired(true);
      return kryo;
      
      
   }

   public ByteBuffer toBuffer()
   {
      ByteArrayOutputStream arrayStream = new ByteArrayOutputStream();
      Output output = new Output(arrayStream);
      getKryo().writeClassAndObject(output, this);
      output.flush();
      byte[] data = arrayStream.toByteArray();
      ByteBuffer direct = ByteBuffer.allocateDirect(data.length + 6);
      direct.putShort(header);
      direct.putInt(data.length);
      direct.put(data);
      direct.flip();
      return direct;
   }
   
   public static LogHandshake read(SelectionKey key) throws IOException
   {
      ByteBuffer header = ByteBuffer.allocate(6);
      while(header.hasRemaining())
      {
         if(StreamingDataTCPClient.selectAndRead(key, header, StreamingDataTCPClient.TIMEOUT) == -1)
         {
            return null;
         }
      }
      header.flip();
      
      short headerShort = header.getShort();
      if(headerShort != LogHandshake.header)
      {
         return null;
      }
      
      int length = header.getInt();
      ByteBuffer data = ByteBuffer.allocateDirect(length);
      while(data.hasRemaining())
      {
         if(StreamingDataTCPClient.selectAndRead(key, data, StreamingDataTCPClient.TIMEOUT) == -1)
         {
            return null;
         }
      }
      data.flip();
      Input input = new Input(new ByteBufferInputStream(data));
      return (LogHandshake) getKryo().readClassAndObject(input);
   }

}
