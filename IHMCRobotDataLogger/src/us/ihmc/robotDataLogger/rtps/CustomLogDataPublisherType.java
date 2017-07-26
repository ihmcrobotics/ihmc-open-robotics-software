package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;
import java.nio.ByteBuffer;

import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.robotDataLogger.LogDataType;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBuffer;
import us.ihmc.tools.compression.CompressionImplementation;
import us.ihmc.tools.compression.CompressionImplementationFactory;

/**
* 
* Topic data type of the struct "LogData" defined in "LogData.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file has been modified from the generated version to provide higher performance.
*
*/
public class CustomLogDataPublisherType implements TopicDataType<RegistrySendBuffer>
{
   public static final String name = "us::ihmc::robotDataLogger::LogData";

   private final int numberOfVariables;
   private final int numberOfStates;

   private final ByteBuffer compressBuffer;
   private final CompressionImplementation compressor;

   public CustomLogDataPublisherType(int numberOfVariables, int numberOfStates)
   {
      this.numberOfVariables = numberOfVariables;
      this.numberOfStates = numberOfStates;

      compressor = CompressionImplementationFactory.instance();
      if (compressor.supportsDirectOutput())
      {
         compressBuffer = null;
      }
      else
      {
         compressBuffer = ByteBuffer.allocate(compressor.maxCompressedLength(numberOfVariables * 8));
      }
   }

   private final CDR serializeCDR = new CDR();

   /**
    * Directly write the compressed data in the serialized buffer. This skips a copy compared to compressing to a temporary buffer
    * 
    * @param databuffer
    * @param serializedPayload
    */
   private void compressDirect(ByteBuffer databuffer, SerializedPayload serializedPayload)
   {
      ByteBuffer serializeBuffer = serializedPayload.getData();
      serializeCDR.write_type_2(0);
      int sizePosition = serializeBuffer.position() - 4;
      int written = compressor.compress(databuffer, serializeBuffer);
      serializeBuffer.putInt(sizePosition, written);
      
   }

   /**
    * If the compression algorithm uses indirect buffers, we need to use a temporary buffer and make copy the compressed data in the serialized payload. 
    * 
    * @param databuffer
    * @param serializedPayload
    * @throws IOException
    */
   private void compressJavaBuffer(ByteBuffer databuffer, SerializedPayload serializedPayload) throws IOException
   {
      compressBuffer.clear();
      compressor.compress(databuffer, compressBuffer);
      compressBuffer.flip();

      // Write compressed data length
      serializeCDR.write_type_2(compressBuffer.remaining());
      serializedPayload.getData().put(compressBuffer);
   }

   @Override
   public void serialize(RegistrySendBuffer data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      serializeCDR.write_type_11(data.getUid());

      serializeCDR.write_type_11(data.getTimestamp());

      serializeCDR.write_type_11(data.getTransmitTime());

      serializeCDR.write_type_c(data.getType().ordinal());

      serializeCDR.write_type_2(data.getRegistryID());
      

      serializeCDR.write_type_2(data.getOffset());
      
      serializeCDR.write_type_2(data.getNumberOfVariables());


      if(data.getType() == LogDataType.DATA_PACKET)
      {
         if (compressor.supportsDirectOutput())
         {
            compressDirect(data.getBuffer(), serializedPayload);
         }
         else
         {
            compressJavaBuffer(data.getBuffer(), serializedPayload);
         }
   
         // Write joint states length
         double[] jointstates = data.getJointStates();
         serializeCDR.write_type_2(jointstates.length);
         for (int i = 0; i < jointstates.length; i++)
         {
            serializeCDR.write_type_6(jointstates[i]);
         }
      }
      else
      {
         serializeCDR.write_type_2(0);
         serializeCDR.write_type_2(0);
      }

      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(SerializedPayload serializedPayload, RegistrySendBuffer data) throws IOException
   {
      throw new RuntimeException("Not implemented");
   }

   @Override
   public final void serialize(RegistrySendBuffer data, InterchangeSerializer ser)
   {
      throw new RuntimeException("Not implemented");

   }

   @Override
   public final void deserialize(InterchangeSerializer ser, RegistrySendBuffer data)
   {
      throw new RuntimeException("Not implemented");

   }

   @Override
   public RegistrySendBuffer createData()
   {
      return null;
   }

   @Override
   public int getTypeSize()
   {
      int rawSize = getTypeSize(compressor.maxCompressedLength(numberOfVariables * 8), numberOfStates);
      if(rawSize > DataProducerParticipant.getMaximumSynchronousPacketSize())
      {
         return DataProducerParticipant.getMaximumSynchronousPacketSize();
      }
      else
      {
         return rawSize;
      }
   }

   public static int getTypeSize(int maxCompressedSize, int numberOfStates)
   {

      int current_alignment = 0;

      current_alignment += 8 + CDR.alignment(current_alignment, 8);

      current_alignment += 8 + CDR.alignment(current_alignment, 8);

      current_alignment += 8 + CDR.alignment(current_alignment, 8);

      current_alignment += 4 + CDR.alignment(current_alignment, 4);

      current_alignment += 4 + CDR.alignment(current_alignment, 4);
      
      current_alignment += 4 + CDR.alignment(current_alignment, 4);

      current_alignment += 4 + CDR.alignment(current_alignment, 4);
      
      current_alignment += 4 + CDR.alignment(current_alignment, 4);
      current_alignment += (maxCompressedSize) + CDR.alignment(current_alignment, 1);

      current_alignment += 4 + CDR.alignment(current_alignment, 4);
      current_alignment += (numberOfStates * 8) + CDR.alignment(current_alignment, 8);

      return CDR.getTypeSize(current_alignment);
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public CustomLogDataPublisherType newInstance()
   {
      return new CustomLogDataPublisherType(numberOfVariables, numberOfStates);
   }

   @Override
   public void serialize(RegistrySendBuffer data, CDR cdr)
   {
      throw new RuntimeException("Not implemented");
   }

   @Override
   public void deserialize(RegistrySendBuffer data, CDR cdr)
   {
      throw new RuntimeException("Not implemented");
   }

   @Override
   public void copy(RegistrySendBuffer src, RegistrySendBuffer dest)
   {
      throw new RuntimeException("Not implemented");
   }
}