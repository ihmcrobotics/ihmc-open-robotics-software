package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.CDR;
import us.ihmc.idl.IDLSequence;

/**
* 
* Topic data type of the struct "YoVariableDefinition" defined in "Handshake.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class YoVariableDefinitionPubSubType implements TopicDataType<us.ihmc.robotDataLogger.YoVariableDefinition>
{
	public static final String name = "us::ihmc::robotDataLogger::YoVariableDefinition";
	
	
	
    public YoVariableDefinitionPubSubType()
    {
        
    }

	private final CDR serializeCDR = new CDR();
	private final CDR deserializeCDR = new CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.YoVariableDefinition data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(SerializedPayload serializedPayload, us.ihmc.robotDataLogger.YoVariableDefinition data) throws IOException
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
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	    current_alignment += 2 + CDR.alignment(current_alignment, 2);

	    current_alignment += 2 + CDR.alignment(current_alignment, 2);

	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.YoVariableDefinition data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.YoVariableDefinition data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	    current_alignment += 2 + CDR.alignment(current_alignment, 2);

	    current_alignment += 2 + CDR.alignment(current_alignment, 2);

	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.YoVariableDefinition data, CDR cdr)
   {

	    if(data.getName().length() <= 255)
	    cdr.write_type_d(data.getName());else
	        throw new RuntimeException("name field exceeds the maximum length");

	    cdr.write_type_c(data.getType().ordinal());


	    cdr.write_type_3(data.getRegistry());

	    cdr.write_type_3(data.getEnumType());

	    cdr.write_type_7(data.getAllowNullValues());
   }

   public static void read(us.ihmc.robotDataLogger.YoVariableDefinition data, CDR cdr)
   {

	    	cdr.read_type_d(data.getName());	

	    	data.setType(us.ihmc.robotDataLogger.YoType.values[cdr.read_type_c()]);
	    	

	    	data.setRegistry(cdr.read_type_3());
	    	

	    	data.setEnumType(cdr.read_type_3());
	    	

	    	data.setAllowNullValues(cdr.read_type_7());
	    	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.YoVariableDefinition data, InterchangeSerializer ser)
	{
			    ser.write_type_d("name", data.getName());
			    
			    ser.write_type_c("type", data.getType());
			    
			    ser.write_type_3("registry", data.getRegistry());
			    
			    ser.write_type_3("enumType", data.getEnumType());
			    
			    ser.write_type_7("allowNullValues", data.getAllowNullValues());
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser, us.ihmc.robotDataLogger.YoVariableDefinition data)
	{
	    			ser.read_type_d("name", data.getName());	
	    	    
	    			data.setType((us.ihmc.robotDataLogger.YoType)ser.read_type_c("type", us.ihmc.robotDataLogger.YoType.class));
	    	
	    	    
	    			data.setRegistry(ser.read_type_3("registry"));	
	    	    
	    			data.setEnumType(ser.read_type_3("enumType"));	
	    	    
	    			data.setAllowNullValues(ser.read_type_7("allowNullValues"));	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.YoVariableDefinition src, us.ihmc.robotDataLogger.YoVariableDefinition dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.YoVariableDefinition createData()
   {
      return new us.ihmc.robotDataLogger.YoVariableDefinition();
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
   
   public void serialize(us.ihmc.robotDataLogger.YoVariableDefinition data, CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.YoVariableDefinition data, CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.YoVariableDefinition src, us.ihmc.robotDataLogger.YoVariableDefinition dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public YoVariableDefinitionPubSubType newInstance()
   {
   	  return new YoVariableDefinitionPubSubType();
   }
}