package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.idl.CDR;

/**
* 
* Topic data type of the struct "ModelFileDescription" defined in "Announcement.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Announcement.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Announcement.idl instead.
*
*/
public class ModelFileDescriptionPubSubType implements TopicDataType<ModelFileDescription>
{
	public static final String name = "us::ihmc::robotDataLogger::ModelFileDescription";
	private final CDR serializeCDR = new CDR();
	private final CDR deserializeCDR = new CDR();
	
	
	
    public ModelFileDescriptionPubSubType()
    {
        
    }
    
       @Override
   public void serialize(ModelFileDescription data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      data.serialize(serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(SerializedPayload serializedPayload, ModelFileDescription data) throws IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      data.deserialize(deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public int getTypeSize()
   {
      return CDR.getTypeSize(ModelFileDescription.getMaxCdrSerializedSize());
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public ModelFileDescription createData()
   {
      return new ModelFileDescription();
   }
   
   @Override
   public ModelFileDescriptionPubSubType newInstance()
   {
   	  return new ModelFileDescriptionPubSubType();
   }
}