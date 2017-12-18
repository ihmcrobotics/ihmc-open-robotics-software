package us.ihmc.robotDataLogger;

/**
* 
* Topic data type of the struct "ModelFileDescription" defined in "Announcement.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Announcement.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Announcement.idl instead.
*
*/
public class ModelFileDescriptionPubSubType implements us.ihmc.pubsub.TopicDataType<us.ihmc.robotDataLogger.ModelFileDescription>
{
	public static final java.lang.String name = "us::ihmc::robotDataLogger::ModelFileDescription";
	
	
	
    public ModelFileDescriptionPubSubType()
    {
        
    }

	private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
	private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.ModelFileDescription data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, us.ihmc.robotDataLogger.ModelFileDescription data) throws java.io.IOException
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

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 255; ++a)
	    {
	        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
	    }
	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.ModelFileDescription data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.ModelFileDescription data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getModelLoaderClass().length() + 1;

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getResourceDirectories().size(); ++a)
	    {
	        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getResourceDirectories().get(a).length() + 1;
	    }
	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	    current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.ModelFileDescription data, us.ihmc.idl.CDR cdr)
   {

	    cdr.write_type_7(data.getHasModel());

	    if(data.getName().length() <= 255)
	    cdr.write_type_d(data.getName());else
	        throw new RuntimeException("name field exceeds the maximum length");

	    if(data.getModelLoaderClass().length() <= 255)
	    cdr.write_type_d(data.getModelLoaderClass());else
	        throw new RuntimeException("modelLoaderClass field exceeds the maximum length");

	    if(data.getResourceDirectories().size() <= 255)
	    cdr.write_type_e(data.getResourceDirectories());else
	        throw new RuntimeException("resourceDirectories field exceeds the maximum length");

	    cdr.write_type_2(data.getModelFileSize());

	    cdr.write_type_7(data.getHasResourceZip());

	    cdr.write_type_2(data.getResourceZipSize());
   }

   public static void read(us.ihmc.robotDataLogger.ModelFileDescription data, us.ihmc.idl.CDR cdr)
   {

	    	data.setHasModel(cdr.read_type_7());
	    	

	    	cdr.read_type_d(data.getName());	

	    	cdr.read_type_d(data.getModelLoaderClass());	

	    	cdr.read_type_e(data.getResourceDirectories());	

	    	data.setModelFileSize(cdr.read_type_2());
	    	

	    	data.setHasResourceZip(cdr.read_type_7());
	    	

	    	data.setResourceZipSize(cdr.read_type_2());
	    	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.ModelFileDescription data, us.ihmc.idl.InterchangeSerializer ser)
	{
			    ser.write_type_7("hasModel", data.getHasModel());
			    
			    ser.write_type_d("name", data.getName());
			    
			    ser.write_type_d("modelLoaderClass", data.getModelLoaderClass());
			    
			    ser.write_type_e("resourceDirectories", data.getResourceDirectories());
			    
			    ser.write_type_2("modelFileSize", data.getModelFileSize());
			    
			    ser.write_type_7("hasResourceZip", data.getHasResourceZip());
			    
			    ser.write_type_2("resourceZipSize", data.getResourceZipSize());
			    
	}
	
	@Override
	public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, us.ihmc.robotDataLogger.ModelFileDescription data)
	{
	    			data.setHasModel(ser.read_type_7("hasModel"));	
	    	    
	    			ser.read_type_d("name", data.getName());	
	    	    
	    			ser.read_type_d("modelLoaderClass", data.getModelLoaderClass());	
	    	    
	    			ser.read_type_e("resourceDirectories", data.getResourceDirectories());	
	    	    
	    			data.setModelFileSize(ser.read_type_2("modelFileSize"));	
	    	    
	    			data.setHasResourceZip(ser.read_type_7("hasResourceZip"));	
	    	    
	    			data.setResourceZipSize(ser.read_type_2("resourceZipSize"));	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.ModelFileDescription src, us.ihmc.robotDataLogger.ModelFileDescription dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.ModelFileDescription createData()
   {
      return new us.ihmc.robotDataLogger.ModelFileDescription();
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
   
   public void serialize(us.ihmc.robotDataLogger.ModelFileDescription data, us.ihmc.idl.CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.ModelFileDescription data, us.ihmc.idl.CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.ModelFileDescription src, us.ihmc.robotDataLogger.ModelFileDescription dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public ModelFileDescriptionPubSubType newInstance()
   {
   	  return new ModelFileDescriptionPubSubType();
   }
}