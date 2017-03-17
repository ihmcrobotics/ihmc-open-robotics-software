package us.ihmc.idl.us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.idl.CDR;

/**
* 
* Topic data type of the struct "YoVariableDefinition" defined in "Handshake.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class YoVariableDefinitionPubSubType implements TopicDataType<YoVariableDefinition>
{
	public static final String name = "us::ihmc::robotDataLogger::YoVariableDefinition";
	private final CDR serializeCDR = new CDR();
	private final CDR deserializeCDR = new CDR();
	
	
	
    public YoVariableDefinitionPubSubType()
    {
        
    }
    
       @Override
   public void serialize(YoVariableDefinition data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      data.serialize(serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(SerializedPayload serializedPayload, YoVariableDefinition data) throws IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      data.deserialize(deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public int getTypeSize()
   {
      return YoVariableDefinition.getMaxCdrSerializedSize();
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public YoVariableDefinition createData()
   {
      return new YoVariableDefinition();
   }
}