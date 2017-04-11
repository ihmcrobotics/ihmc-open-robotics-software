package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.idl.CDR;

/**
* 
* Topic data type of the struct "Timestamp" defined in "Timestamp.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Timestamp.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Timestamp.idl instead.
*
*/
public class TimestampPubSubType implements TopicDataType<Timestamp>
{
	public static final String name = "us::ihmc::robotDataLogger::Timestamp";
	private final CDR serializeCDR = new CDR();
	private final CDR deserializeCDR = new CDR();
	
	
	
    public TimestampPubSubType()
    {
        
    }
    
       @Override
   public void serialize(Timestamp data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      data.serialize(serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(SerializedPayload serializedPayload, Timestamp data) throws IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      data.deserialize(deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public int getTypeSize()
   {
      return CDR.getTypeSize(Timestamp.getMaxCdrSerializedSize());
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public Timestamp createData()
   {
      return new Timestamp();
   }
   
   @Override
   public TimestampPubSubType newInstance()
   {
   	  return new TimestampPubSubType();
   }
}