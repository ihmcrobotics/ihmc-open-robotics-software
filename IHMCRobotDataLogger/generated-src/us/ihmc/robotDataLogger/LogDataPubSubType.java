package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.idl.CDR;

/**
* 
* Topic data type of the struct "LogData" defined in "LogData.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LogData.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogData.idl instead.
*
*/
public class LogDataPubSubType implements TopicDataType<LogData>
{
	public static final String name = "us::ihmc::robotDataLogger::LogData";
	private final CDR serializeCDR = new CDR();
	private final CDR deserializeCDR = new CDR();
	
	
	
    public LogDataPubSubType()
    {
        
    }
    
       @Override
   public void serialize(LogData data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      data.serialize(serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(SerializedPayload serializedPayload, LogData data) throws IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      data.deserialize(deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public int getTypeSize()
   {
      return LogData.getMaxCdrSerializedSize();
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public LogData createData()
   {
      return new LogData();
   }
}