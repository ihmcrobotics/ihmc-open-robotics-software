package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;
import java.nio.ByteBuffer;

import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.robotDataLogger.LogDataType;
import us.ihmc.robotDataLogger.dataBuffers.RegistryReceiveBuffer;
import us.ihmc.tools.compression.CompressionImplementation;
import us.ihmc.tools.compression.CompressionImplementationFactory;

/**
* 
* Topic data type of the struct "LogData" defined in "LogData.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file has been modified from the generated version to provide higher performance.
*
*/
public class CustomLogDataSubscriberType implements TopicDataType<RegistryReceiveBuffer>
{
   public static final String name = "us::ihmc::robotDataLogger::LogData";

   private final int numberOfVariables;
   private final int numberOfStates;

   private final CompressionImplementation compressor;

   public CustomLogDataSubscriberType(int maxNumberOfVariables, int maxNumberOfStates)
   {
      this.numberOfVariables = maxNumberOfVariables;
      this.numberOfStates = maxNumberOfStates;

      compressor = CompressionImplementationFactory.instance();
   }

   private final CDR deserializeCDR = new CDR();

   @Override
   public void serialize(RegistryReceiveBuffer data, SerializedPayload serializedPayload) throws IOException
   {
      throw new RuntimeException("Not implemented");
   }

   @Override
   public void deserialize(SerializedPayload serializedPayload, RegistryReceiveBuffer data) throws IOException
   {
      deserializeCDR.deserialize(serializedPayload);

      data.setUid(deserializeCDR.read_type_11());

      data.setTimestamp(deserializeCDR.read_type_11());
      
      data.setTransmitTime(deserializeCDR.read_type_11());


      data.setType(us.ihmc.robotDataLogger.LogDataType.values[deserializeCDR.read_type_c()]);
      
      data.setRegistryID(deserializeCDR.read_type_2());
      
      data.setOffset(deserializeCDR.read_type_2());
      
      data.setNumberOfVariables(deserializeCDR.read_type_2());
      
      if(data.getType() == LogDataType.DATA_PACKET)
      {
         int dataLength = deserializeCDR.read_type_2();
         ByteBuffer buffer = data.allocateBuffer(dataLength);
         serializedPayload.getData().get(buffer.array(), 0, dataLength);
         buffer.limit(dataLength);
      
         int stateLength = deserializeCDR.read_type_2();
         double[] states = data.allocateStates(stateLength);
         for (int i = 0; i < stateLength; i++)
         {
            states[i] = deserializeCDR.read_type_6();
         }
      }

      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(RegistryReceiveBuffer data, InterchangeSerializer ser)
   {
      throw new RuntimeException("Not implemented");

   }

   @Override
   public final void deserialize(InterchangeSerializer ser, RegistryReceiveBuffer data)
   {
      throw new RuntimeException("Not implemented");

   }

   @Override
   public RegistryReceiveBuffer createData()
   {
      return null;
   }

   @Override
   public int getTypeSize()
   {
      int rawSize = CustomLogDataPublisherType.getTypeSize(compressor.maxCompressedLength(numberOfVariables * 8), numberOfStates);
      if(rawSize > DataProducerParticipant.getMaximumSynchronousPacketSize())
      {
         return DataProducerParticipant.getMaximumSynchronousPacketSize();
      }
      else
      {
         return rawSize;
      }
   }


   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public CustomLogDataSubscriberType newInstance()
   {
      return new CustomLogDataSubscriberType(numberOfVariables, numberOfStates);
   }

   @Override
   public void serialize(RegistryReceiveBuffer data, CDR cdr)
   {
      throw new RuntimeException("Not implemented");
   }

   @Override
   public void deserialize(RegistryReceiveBuffer data, CDR cdr)
   {
      throw new RuntimeException("Not implemented");
   }

   @Override
   public void copy(RegistryReceiveBuffer src, RegistryReceiveBuffer dest)
   {
      throw new RuntimeException("Not implemented");
   }
}