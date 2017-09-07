package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.CDR;
import us.ihmc.idl.IDLSequence;

/**
* 
* Topic data type of the struct "VariableChangeRequest" defined in "VariableChangeRequest.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from VariableChangeRequest.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit VariableChangeRequest.idl instead.
*
*/
public class VariableChangeRequestPubSubType implements TopicDataType<us.ihmc.robotDataLogger.VariableChangeRequest>
{
	public static final String name = "us::ihmc::robotDataLogger::VariableChangeRequest";
	
	
	
    public VariableChangeRequestPubSubType()
    {
        
    }

	private final CDR serializeCDR = new CDR();
	private final CDR deserializeCDR = new CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.VariableChangeRequest data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(SerializedPayload serializedPayload, us.ihmc.robotDataLogger.VariableChangeRequest data) throws IOException
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
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.VariableChangeRequest data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.VariableChangeRequest data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.VariableChangeRequest data, CDR cdr)
   {

	    cdr.write_type_2(data.getVariableID());

	    cdr.write_type_6(data.getRequestedValue());
   }

   public static void read(us.ihmc.robotDataLogger.VariableChangeRequest data, CDR cdr)
   {

	    	data.setVariableID(cdr.read_type_2());
	    	

	    	data.setRequestedValue(cdr.read_type_6());
	    	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.VariableChangeRequest data, InterchangeSerializer ser)
	{
			    ser.write_type_2("variableID", data.getVariableID());
			    
			    ser.write_type_6("requestedValue", data.getRequestedValue());
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser, us.ihmc.robotDataLogger.VariableChangeRequest data)
	{
	    			data.setVariableID(ser.read_type_2("variableID"));	
	    	    
	    			data.setRequestedValue(ser.read_type_6("requestedValue"));	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.VariableChangeRequest src, us.ihmc.robotDataLogger.VariableChangeRequest dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.VariableChangeRequest createData()
   {
      return new us.ihmc.robotDataLogger.VariableChangeRequest();
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
   
   public void serialize(us.ihmc.robotDataLogger.VariableChangeRequest data, CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.VariableChangeRequest data, CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.VariableChangeRequest src, us.ihmc.robotDataLogger.VariableChangeRequest dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public VariableChangeRequestPubSubType newInstance()
   {
   	  return new VariableChangeRequestPubSubType();
   }
}