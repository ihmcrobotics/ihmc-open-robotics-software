package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.idl.CDR;

/**
* 
* Topic data type of the struct "Model" defined in "LogProperties.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LogProperties.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogProperties.idl instead.
*
*/
public class ModelPubSubType implements TopicDataType<Model>
{
	public static final String name = "us::ihmc::robotDataLogger::Model";
	private final CDR serializeCDR = new CDR();
	private final CDR deserializeCDR = new CDR();
	
	
	
    public ModelPubSubType()
    {
        
    }
    
       @Override
   public void serialize(Model data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      data.serialize(serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(SerializedPayload serializedPayload, Model data) throws IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      data.deserialize(deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public int getTypeSize()
   {
      return CDR.getTypeSize(Model.getMaxCdrSerializedSize());
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public Model createData()
   {
      return new Model();
   }
   
   @Override
   public ModelPubSubType newInstance()
   {
   	  return new ModelPubSubType();
   }
}