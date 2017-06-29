package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.CDR;
import us.ihmc.idl.IDLSequence;

/**
* 
* Topic data type of the struct "JointDefinition" defined in "Handshake.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class JointDefinitionPubSubType implements TopicDataType<us.ihmc.robotDataLogger.JointDefinition>
{
	public static final String name = "us::ihmc::robotDataLogger::JointDefinition";
	
	
	
    public JointDefinitionPubSubType()
    {
        
    }

	private final CDR serializeCDR = new CDR();
	private final CDR deserializeCDR = new CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.JointDefinition data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(SerializedPayload serializedPayload, us.ihmc.robotDataLogger.JointDefinition data) throws IOException
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

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.JointDefinition data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.JointDefinition data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.JointDefinition data, CDR cdr)
   {

	    if(data.getName().length() <= 255)
	    cdr.write_type_d(data.getName());else
	        throw new RuntimeException("name field exceeds the maximum length");

	    cdr.write_type_c(data.getType().ordinal());

   }

   public static void read(us.ihmc.robotDataLogger.JointDefinition data, CDR cdr)
   {

	    	cdr.read_type_d(data.getName());	

	    	data.setType(us.ihmc.robotDataLogger.JointType.values[cdr.read_type_c()]);
	    	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.JointDefinition data, InterchangeSerializer ser)
	{
			    ser.write_type_d("name", data.getName());
			    
			    ser.write_type_c("type", data.getType());
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser, us.ihmc.robotDataLogger.JointDefinition data)
	{
	    			ser.read_type_d("name", data.getName());	
	    	    
	    			data.setType((us.ihmc.robotDataLogger.JointType)ser.read_type_c("type", us.ihmc.robotDataLogger.JointType.class));
	    	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.JointDefinition src, us.ihmc.robotDataLogger.JointDefinition dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.JointDefinition createData()
   {
      return new us.ihmc.robotDataLogger.JointDefinition();
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
   
   public void serialize(us.ihmc.robotDataLogger.JointDefinition data, CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.JointDefinition data, CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.JointDefinition src, us.ihmc.robotDataLogger.JointDefinition dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public JointDefinitionPubSubType newInstance()
   {
   	  return new JointDefinitionPubSubType();
   }
}