package us.ihmc.robotDataLogger;

/**
* 
* Topic data type of the struct "Summary" defined in "Handshake.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class SummaryPubSubType implements us.ihmc.pubsub.TopicDataType<us.ihmc.robotDataLogger.Summary>
{
	public static final java.lang.String name = "us::ihmc::robotDataLogger::Summary";
	
	
	
    public SummaryPubSubType()
    {
        
    }

	private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
	private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.Summary data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, us.ihmc.robotDataLogger.Summary data) throws java.io.IOException
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
	            
	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 1024 + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 128; ++a)
	    {
	        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 1024 + 1;
	    }
	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Summary data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Summary data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getSummaryTriggerVariable().length() + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getSummarizedVariables().size(); ++a)
	    {
	        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getSummarizedVariables().get(a).length() + 1;
	    }
	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.Summary data, us.ihmc.idl.CDR cdr)
   {

	    cdr.write_type_7(data.getCreateSummary());

	    if(data.getSummaryTriggerVariable().length() <= 1024)
	    cdr.write_type_d(data.getSummaryTriggerVariable());else
	        throw new RuntimeException("summaryTriggerVariable field exceeds the maximum length");

	    if(data.getSummarizedVariables().size() <= 128)
	    cdr.write_type_e(data.getSummarizedVariables());else
	        throw new RuntimeException("summarizedVariables field exceeds the maximum length");
   }

   public static void read(us.ihmc.robotDataLogger.Summary data, us.ihmc.idl.CDR cdr)
   {

	    	data.setCreateSummary(cdr.read_type_7());
	    	

	    	cdr.read_type_d(data.getSummaryTriggerVariable());	

	    	cdr.read_type_e(data.getSummarizedVariables());	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.Summary data, us.ihmc.idl.InterchangeSerializer ser)
	{
			    ser.write_type_7("createSummary", data.getCreateSummary());
			    
			    ser.write_type_d("summaryTriggerVariable", data.getSummaryTriggerVariable());
			    
			    ser.write_type_e("summarizedVariables", data.getSummarizedVariables());
			    
	}
	
	@Override
	public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, us.ihmc.robotDataLogger.Summary data)
	{
	    			data.setCreateSummary(ser.read_type_7("createSummary"));	
	    	    
	    			ser.read_type_d("summaryTriggerVariable", data.getSummaryTriggerVariable());	
	    	    
	    			ser.read_type_e("summarizedVariables", data.getSummarizedVariables());	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.Summary src, us.ihmc.robotDataLogger.Summary dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.Summary createData()
   {
      return new us.ihmc.robotDataLogger.Summary();
   }
      

   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }
   
   public void serialize(us.ihmc.robotDataLogger.Summary data, us.ihmc.idl.CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.Summary data, us.ihmc.idl.CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.Summary src, us.ihmc.robotDataLogger.Summary dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public SummaryPubSubType newInstance()
   {
   	  return new SummaryPubSubType();
   }
}