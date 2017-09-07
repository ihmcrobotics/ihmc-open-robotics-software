package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.CDR;
import us.ihmc.idl.IDLSequence;

/**
* 
* Topic data type of the struct "AppearanceDefinitionMessage" defined in "Handshake.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class AppearanceDefinitionMessagePubSubType implements TopicDataType<us.ihmc.robotDataLogger.AppearanceDefinitionMessage>
{
	public static final String name = "us::ihmc::robotDataLogger::AppearanceDefinitionMessage";
	
	
	
    public AppearanceDefinitionMessagePubSubType()
    {
        
    }

	private final CDR serializeCDR = new CDR();
	private final CDR deserializeCDR = new CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.AppearanceDefinitionMessage data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(SerializedPayload serializedPayload, us.ihmc.robotDataLogger.AppearanceDefinitionMessage data) throws IOException
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

	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.AppearanceDefinitionMessage data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.AppearanceDefinitionMessage data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.AppearanceDefinitionMessage data, CDR cdr)
   {

	    cdr.write_type_6(data.getR());

	    cdr.write_type_6(data.getG());

	    cdr.write_type_6(data.getB());

	    cdr.write_type_6(data.getTransparency());
   }

   public static void read(us.ihmc.robotDataLogger.AppearanceDefinitionMessage data, CDR cdr)
   {

	    	data.setR(cdr.read_type_6());
	    	

	    	data.setG(cdr.read_type_6());
	    	

	    	data.setB(cdr.read_type_6());
	    	

	    	data.setTransparency(cdr.read_type_6());
	    	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.AppearanceDefinitionMessage data, InterchangeSerializer ser)
	{
			    ser.write_type_6("r", data.getR());
			    
			    ser.write_type_6("g", data.getG());
			    
			    ser.write_type_6("b", data.getB());
			    
			    ser.write_type_6("transparency", data.getTransparency());
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser, us.ihmc.robotDataLogger.AppearanceDefinitionMessage data)
	{
	    			data.setR(ser.read_type_6("r"));	
	    	    
	    			data.setG(ser.read_type_6("g"));	
	    	    
	    			data.setB(ser.read_type_6("b"));	
	    	    
	    			data.setTransparency(ser.read_type_6("transparency"));	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.AppearanceDefinitionMessage src, us.ihmc.robotDataLogger.AppearanceDefinitionMessage dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.AppearanceDefinitionMessage createData()
   {
      return new us.ihmc.robotDataLogger.AppearanceDefinitionMessage();
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
   
   public void serialize(us.ihmc.robotDataLogger.AppearanceDefinitionMessage data, CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.AppearanceDefinitionMessage data, CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.AppearanceDefinitionMessage src, us.ihmc.robotDataLogger.AppearanceDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public AppearanceDefinitionMessagePubSubType newInstance()
   {
   	  return new AppearanceDefinitionMessagePubSubType();
   }
}