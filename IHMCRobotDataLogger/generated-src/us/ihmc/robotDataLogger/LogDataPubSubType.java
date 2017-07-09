package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.CDR;
import us.ihmc.idl.IDLSequence;

/**
* 
* Topic data type of the struct "LogData" defined in "LogData.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LogData.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogData.idl instead.
*
*/
public class LogDataPubSubType implements TopicDataType<us.ihmc.robotDataLogger.LogData>
{
	public static final String name = "us::ihmc::robotDataLogger::LogData";
	
	
	
    public LogDataPubSubType()
    {
        
    }

	private final CDR serializeCDR = new CDR();
	private final CDR deserializeCDR = new CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.LogData data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(SerializedPayload serializedPayload, us.ihmc.robotDataLogger.LogData data) throws IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }
   
	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    current_alignment += (100 * 1) + CDR.alignment(current_alignment, 1);


	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.LogData data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.LogData data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    current_alignment += (data.getData().size() * 1) + CDR.alignment(current_alignment, 1);


	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.LogData data, CDR cdr)
   {

	    cdr.write_type_11(data.getUid());

	    cdr.write_type_11(data.getTimestamp());

	    if(data.getData().size() <= 100)
	    cdr.write_type_e(data.getData());else
	        throw new RuntimeException("data field exceeds the maximum length");
   }

   public static void read(us.ihmc.robotDataLogger.LogData data, CDR cdr)
   {

	    	data.setUid(cdr.read_type_11());
	    	

	    	data.setTimestamp(cdr.read_type_11());
	    	

	    	cdr.read_type_e(data.getData());	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.LogData data, InterchangeSerializer ser)
	{
			    ser.write_type_11("uid", data.getUid());
			    
			    ser.write_type_11("timestamp", data.getTimestamp());
			    
			    ser.write_type_e("data", data.getData());
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser, us.ihmc.robotDataLogger.LogData data)
	{
	    			data.setUid(ser.read_type_11("uid"));	
	    	    
	    			data.setTimestamp(ser.read_type_11("timestamp"));	
	    	    
	    			ser.read_type_e("data", data.getData());	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.LogData src, us.ihmc.robotDataLogger.LogData dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.LogData createData()
   {
      return new us.ihmc.robotDataLogger.LogData();
   }
      

   @Override
   public int getTypeSize()
   {
      return CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public String getName()
   {
      return name;
   }
   
   public void serialize(us.ihmc.robotDataLogger.LogData data, CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.LogData data, CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.LogData src, us.ihmc.robotDataLogger.LogData dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public LogDataPubSubType newInstance()
   {
   	  return new LogDataPubSubType();
   }
}