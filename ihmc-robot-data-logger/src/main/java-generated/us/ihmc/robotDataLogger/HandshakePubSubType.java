package us.ihmc.robotDataLogger;

/**
* 
* Topic data type of the struct "Handshake" defined in "Handshake.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class HandshakePubSubType implements us.ihmc.pubsub.TopicDataType<us.ihmc.robotDataLogger.Handshake>
{
	public static final java.lang.String name = "us::ihmc::robotDataLogger::Handshake";
	
	
	
    public HandshakePubSubType()
    {
        
    }

	private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
	private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.Handshake data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, us.ihmc.robotDataLogger.Handshake data) throws java.io.IOException
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
	            
	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 1024; ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.YoRegistryDefinitionPubSubType.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 32767; ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.YoVariableDefinitionPubSubType.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 128; ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.JointDefinitionPubSubType.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 2048; ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.GraphicObjectMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 2048; ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.GraphicObjectMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 1024; ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.EnumTypePubSubType.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += us.ihmc.robotDataLogger.SummaryPubSubType.getMaxCdrSerializedSize(current_alignment);
	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Handshake data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Handshake data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getRegistries().size(); ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.YoRegistryDefinitionPubSubType.getCdrSerializedSize(data.getRegistries().get(a), current_alignment);}

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getVariables().size(); ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.YoVariableDefinitionPubSubType.getCdrSerializedSize(data.getVariables().get(a), current_alignment);}

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getJoints().size(); ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.JointDefinitionPubSubType.getCdrSerializedSize(data.getJoints().get(a), current_alignment);}

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getGraphicObjects().size(); ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.GraphicObjectMessagePubSubType.getCdrSerializedSize(data.getGraphicObjects().get(a), current_alignment);}

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getArtifacts().size(); ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.GraphicObjectMessagePubSubType.getCdrSerializedSize(data.getArtifacts().get(a), current_alignment);}

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getEnumTypes().size(); ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.EnumTypePubSubType.getCdrSerializedSize(data.getEnumTypes().get(a), current_alignment);}

	    current_alignment += us.ihmc.robotDataLogger.SummaryPubSubType.getCdrSerializedSize(data.getSummary(), current_alignment);
	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.Handshake data, us.ihmc.idl.CDR cdr)
   {

	    cdr.write_type_6(data.getDt());

	    if(data.getRegistries().size() <= 1024)
	    cdr.write_type_e(data.getRegistries());else
	        throw new RuntimeException("registries field exceeds the maximum length");

	    if(data.getVariables().size() <= 32767)
	    cdr.write_type_e(data.getVariables());else
	        throw new RuntimeException("variables field exceeds the maximum length");

	    if(data.getJoints().size() <= 128)
	    cdr.write_type_e(data.getJoints());else
	        throw new RuntimeException("joints field exceeds the maximum length");

	    if(data.getGraphicObjects().size() <= 2048)
	    cdr.write_type_e(data.getGraphicObjects());else
	        throw new RuntimeException("graphicObjects field exceeds the maximum length");

	    if(data.getArtifacts().size() <= 2048)
	    cdr.write_type_e(data.getArtifacts());else
	        throw new RuntimeException("artifacts field exceeds the maximum length");

	    if(data.getEnumTypes().size() <= 1024)
	    cdr.write_type_e(data.getEnumTypes());else
	        throw new RuntimeException("enumTypes field exceeds the maximum length");

	    us.ihmc.robotDataLogger.SummaryPubSubType.write(data.getSummary(), cdr);
   }

   public static void read(us.ihmc.robotDataLogger.Handshake data, us.ihmc.idl.CDR cdr)
   {

	    	data.setDt(cdr.read_type_6());
	    	

	    	cdr.read_type_e(data.getRegistries());	

	    	cdr.read_type_e(data.getVariables());	

	    	cdr.read_type_e(data.getJoints());	

	    	cdr.read_type_e(data.getGraphicObjects());	

	    	cdr.read_type_e(data.getArtifacts());	

	    	cdr.read_type_e(data.getEnumTypes());	

	    	us.ihmc.robotDataLogger.SummaryPubSubType.read(data.getSummary(), cdr);	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.Handshake data, us.ihmc.idl.InterchangeSerializer ser)
	{
			    ser.write_type_6("dt", data.getDt());
			    
			    ser.write_type_e("registries", data.getRegistries());
			    
			    ser.write_type_e("variables", data.getVariables());
			    
			    ser.write_type_e("joints", data.getJoints());
			    
			    ser.write_type_e("graphicObjects", data.getGraphicObjects());
			    
			    ser.write_type_e("artifacts", data.getArtifacts());
			    
			    ser.write_type_e("enumTypes", data.getEnumTypes());
			    
			    ser.write_type_a("summary", new us.ihmc.robotDataLogger.SummaryPubSubType(), data.getSummary());

			    
	}
	
	@Override
	public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, us.ihmc.robotDataLogger.Handshake data)
	{
	    			data.setDt(ser.read_type_6("dt"));	
	    	    
	    			ser.read_type_e("registries", data.getRegistries());	
	    	    
	    			ser.read_type_e("variables", data.getVariables());	
	    	    
	    			ser.read_type_e("joints", data.getJoints());	
	    	    
	    			ser.read_type_e("graphicObjects", data.getGraphicObjects());	
	    	    
	    			ser.read_type_e("artifacts", data.getArtifacts());	
	    	    
	    			ser.read_type_e("enumTypes", data.getEnumTypes());	
	    	    
	    			ser.read_type_a("summary", new us.ihmc.robotDataLogger.SummaryPubSubType(), data.getSummary());
	    	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.Handshake src, us.ihmc.robotDataLogger.Handshake dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.Handshake createData()
   {
      return new us.ihmc.robotDataLogger.Handshake();
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
   
   public void serialize(us.ihmc.robotDataLogger.Handshake data, us.ihmc.idl.CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.Handshake data, us.ihmc.idl.CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.Handshake src, us.ihmc.robotDataLogger.Handshake dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public HandshakePubSubType newInstance()
   {
   	  return new HandshakePubSubType();
   }
}