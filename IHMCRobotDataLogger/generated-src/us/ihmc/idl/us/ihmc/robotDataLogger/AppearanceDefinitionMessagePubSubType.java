package us.ihmc.idl.us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.idl.CDR;

/**
* 
* Topic data type of the struct "AppearanceDefinitionMessage" defined in "Handshake.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class AppearanceDefinitionMessagePubSubType implements TopicDataType<AppearanceDefinitionMessage>
{
	public static final String name = "us::ihmc::robotDataLogger::AppearanceDefinitionMessage";
	private final CDR serializeCDR = new CDR();
	private final CDR deserializeCDR = new CDR();
	
	
	
    public AppearanceDefinitionMessagePubSubType()
    {
        
    }
    
       @Override
   public void serialize(AppearanceDefinitionMessage data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      data.serialize(serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(SerializedPayload serializedPayload, AppearanceDefinitionMessage data) throws IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      data.deserialize(deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public int getTypeSize()
   {
      return AppearanceDefinitionMessage.getMaxCdrSerializedSize();
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public AppearanceDefinitionMessage createData()
   {
      return new AppearanceDefinitionMessage();
   }
}