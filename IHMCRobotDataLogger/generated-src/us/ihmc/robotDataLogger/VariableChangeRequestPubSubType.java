package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.idl.CDR;

/**
* 
* Topic data type of the struct "VariableChangeRequest" defined in "VariableChangeRequest.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from VariableChangeRequest.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit VariableChangeRequest.idl instead.
*
*/
public class VariableChangeRequestPubSubType implements TopicDataType<VariableChangeRequest>
{
	public static final String name = "us::ihmc::robotDataLogger::VariableChangeRequest";
	private final CDR serializeCDR = new CDR();
	private final CDR deserializeCDR = new CDR();
	
	
	
    public VariableChangeRequestPubSubType()
    {
        
    }
    
       @Override
   public void serialize(VariableChangeRequest data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      data.serialize(serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(SerializedPayload serializedPayload, VariableChangeRequest data) throws IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      data.deserialize(deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public int getTypeSize()
   {
      return VariableChangeRequest.getMaxCdrSerializedSize();
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public VariableChangeRequest createData()
   {
      return new VariableChangeRequest();
   }
}