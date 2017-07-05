package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;
import java.nio.ByteBuffer;

import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.robotDataLogger.LogDataType;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBuffer;
import us.ihmc.tools.compression.SnappyUtils;

/**
* 
* Topic data type of the struct "LogData" defined in "LogData.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file has been modified from the generated version to provide higher performance.
*
*/
public class CustomLogDataPubisherType implements TopicDataType<RegistrySendBuffer>
{
   public static final String name = "us::ihmc::robotDataLogger::LogData";

   private final int numberOfVariables;
   private final int numberOfStates;
   private final int maximumCompressedSize;

   private final ByteBuffer compressBuffer;

   public CustomLogDataPubisherType(int numberOfVariables, int numberOfStates)
   {
      this.numberOfVariables = numberOfVariables;
      this.numberOfStates = numberOfStates;

      maximumCompressedSize = SnappyUtils.maxCompressedLength(numberOfVariables * 8);

      compressBuffer = ByteBuffer.allocate(maximumCompressedSize);
   }

   private final CDR serializeCDR = new CDR();

   @Override
   public void serialize(RegistrySendBuffer data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      serializeCDR.write_type_11(data.getUid());

      serializeCDR.write_type_11(data.getTimestamp());

      serializeCDR.write_type_c(LogDataType.DATA_PACKET.ordinal());

      serializeCDR.write_type_2(data.getRegistryID());

      ByteBuffer databuffer = data.getBuffer();
      databuffer.clear();
      compressBuffer.clear();
      SnappyUtils.compress(databuffer, compressBuffer);
      compressBuffer.flip();

      // Write compressed data length
      serializeCDR.write_type_2(compressBuffer.remaining());
      serializedPayload.getData().put(compressBuffer);

      // Write joint states length
      double[] jointstates = data.getJointStates();
      serializeCDR.write_type_2(jointstates.length);
      for (int i = 0; i < jointstates.length; i++)
      {
         serializeCDR.write_type_6(jointstates[i]);
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
      int current_alignment = 0;

      current_alignment += 8 + CDR.alignment(current_alignment, 8);

      current_alignment += 8 + CDR.alignment(current_alignment, 8);

      current_alignment += 4 + CDR.alignment(current_alignment, 4);

      current_alignment += 4 + CDR.alignment(current_alignment, 4);

      current_alignment += 4 + CDR.alignment(current_alignment, 4);
      current_alignment += (maximumCompressedSize * 1) + CDR.alignment(current_alignment, 1);

      current_alignment += 4 + CDR.alignment(current_alignment, 4);
      current_alignment += (numberOfStates * 8) + CDR.alignment(current_alignment, 8);

      return current_alignment;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public CustomLogDataPubisherType newInstance()
   {
      return new CustomLogDataPubisherType(numberOfVariables, numberOfStates);
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