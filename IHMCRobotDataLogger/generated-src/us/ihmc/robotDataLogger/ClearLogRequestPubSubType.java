package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.idl.CDR;

/**
* 
* Topic data type of the struct "ClearLogRequest" defined in "ClearLogRequest.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ClearLogRequest.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ClearLogRequest.idl instead.
*
*/
public class ClearLogRequestPubSubType implements TopicDataType<ClearLogRequest>
{
	public static final String name = "us::ihmc::robotDataLogger::ClearLogRequest";
	private final CDR serializeCDR = new CDR();
	private final CDR deserializeCDR = new CDR();
	
	
	
    public ClearLogRequestPubSubType()
    {
        
    }
    
       @Override
   public void serialize(ClearLogRequest data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      data.serialize(serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(SerializedPayload serializedPayload, ClearLogRequest data) throws IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      data.deserialize(deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public int getTypeSize()
   {
      return CDR.getTypeSize(ClearLogRequest.getMaxCdrSerializedSize());
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public ClearLogRequest createData()
   {
      return new ClearLogRequest();
   }
   
   @Override
   public ClearLogRequestPubSubType newInstance()
   {
   	  return new ClearLogRequestPubSubType();
   }
}