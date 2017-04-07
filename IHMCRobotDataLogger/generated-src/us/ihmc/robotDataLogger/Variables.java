package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "Variables" defined in LogProperties.idl. 
*
* This file was automatically generated from LogProperties.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogProperties.idl instead.
*
*/
public class Variables implements IDLStruct<Variables>
{
    public Variables()
    {
        	handshake_ = new StringBuilder(255); 
        	data_ = new StringBuilder(255); 
        	summary_ = new StringBuilder(255); 
        	index_ = new StringBuilder(255); 
        
        
    }
    @Override
    public void set(Variables other)
    {
        	handshakeFileType_ = other.handshakeFileType_;
        	handshake_.setLength(0);
        	handshake_.append(other.handshake_);
        	data_.setLength(0);
        	data_.append(other.data_);
        	summary_.setLength(0);
        	summary_.append(other.summary_);
        	index_.setLength(0);
        	index_.append(other.index_);
        	timestamped_ = other.timestamped_;
        	compressed_ = other.compressed_;

    }

    public void setHandshakeFileType(us.ihmc.robotDataLogger.HandshakeFileType handshakeFileType)
    {
        handshakeFileType_ = handshakeFileType;
    }

    public us.ihmc.robotDataLogger.HandshakeFileType getHandshakeFileType()
    {
        return handshakeFileType_;
    }

        
        public void setHandshake(String handshake)
        {
        	handshake_.setLength(0);
        	handshake_.append(handshake);
        }
        
        public String getHandshakeAsString()
        {
        	return getHandshake().toString();
        }

    public StringBuilder getHandshake()
    {
        return handshake_;
    }

        
        public void setData(String data)
        {
        	data_.setLength(0);
        	data_.append(data);
        }
        
        public String getDataAsString()
        {
        	return getData().toString();
        }

    public StringBuilder getData()
    {
        return data_;
    }

        
        public void setSummary(String summary)
        {
        	summary_.setLength(0);
        	summary_.append(summary);
        }
        
        public String getSummaryAsString()
        {
        	return getSummary().toString();
        }

    public StringBuilder getSummary()
    {
        return summary_;
    }

        
        public void setIndex(String index)
        {
        	index_.setLength(0);
        	index_.append(index);
        }
        
        public String getIndexAsString()
        {
        	return getIndex().toString();
        }

    public StringBuilder getIndex()
    {
        return index_;
    }

        
    public void setTimestamped(boolean timestamped)
    {
        timestamped_ = timestamped;
    }

    public boolean getTimestamped()
    {
        return timestamped_;
    }

        
    public void setCompressed(boolean compressed)
    {
        compressed_ = compressed;
    }

    public boolean getCompressed()
    {
        return compressed_;
    }

        


	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(Variables data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(Variables data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getHandshake().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getData().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getSummary().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getIndex().length() + 1;

	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    cdr.write_type_c(handshakeFileType_.ordinal());


	    if(handshake_.length() <= 255)
	    cdr.write_type_d(handshake_);else
	        throw new RuntimeException("handshake field exceeds the maximum length");

	    if(data_.length() <= 255)
	    cdr.write_type_d(data_);else
	        throw new RuntimeException("data field exceeds the maximum length");

	    if(summary_.length() <= 255)
	    cdr.write_type_d(summary_);else
	        throw new RuntimeException("summary field exceeds the maximum length");

	    if(index_.length() <= 255)
	    cdr.write_type_d(index_);else
	        throw new RuntimeException("index field exceeds the maximum length");

	    cdr.write_type_7(timestamped_);

	    cdr.write_type_7(compressed_);
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	handshakeFileType_ = us.ihmc.robotDataLogger.HandshakeFileType.values[cdr.read_type_c()];
	    	

	    	cdr.read_type_d(handshake_);	

	    	cdr.read_type_d(data_);	

	    	cdr.read_type_d(summary_);	

	    	cdr.read_type_d(index_);	

	    	timestamped_ = cdr.read_type_7();	

	    	compressed_ = cdr.read_type_7();	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_c("handshakeFileType", handshakeFileType_);
			    
			    ser.write_type_d("handshake", handshake_);
			    
			    ser.write_type_d("data", data_);
			    
			    ser.write_type_d("summary", summary_);
			    
			    ser.write_type_d("index", index_);
			    
			    ser.write_type_7("timestamped", timestamped_);
			    
			    ser.write_type_7("compressed", compressed_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			handshakeFileType_ = (us.ihmc.robotDataLogger.HandshakeFileType)ser.read_type_c("handshakeFileType", us.ihmc.robotDataLogger.HandshakeFileType.class);
	    	
	    	    
	    			ser.read_type_d("handshake", handshake_);	
	    	    
	    			ser.read_type_d("data", data_);	
	    	    
	    			ser.read_type_d("summary", summary_);	
	    	    
	    			ser.read_type_d("index", index_);	
	    	    
	    			timestamped_ = ser.read_type_7("timestamped");	
	    	    
	    			compressed_ = ser.read_type_7("compressed");	
	    	    
	}

    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof Variables)) return false;
        Variables otherMyClass = (Variables)other;
        boolean returnedValue = true;

        returnedValue &= this.handshakeFileType_ == otherMyClass.handshakeFileType_;

                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.handshake_, otherMyClass.handshake_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.data_, otherMyClass.data_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.summary_, otherMyClass.summary_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.index_, otherMyClass.index_);
                
        returnedValue &= this.timestamped_ == otherMyClass.timestamped_;

                
        returnedValue &= this.compressed_ == otherMyClass.compressed_;

                

        return returnedValue;
    }
    
     @Override
    public String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("Variables {");
        builder.append("handshakeFileType=");
        builder.append(this.handshakeFileType_);

                builder.append(", ");
        builder.append("handshake=");
        builder.append(this.handshake_);

                builder.append(", ");
        builder.append("data=");
        builder.append(this.data_);

                builder.append(", ");
        builder.append("summary=");
        builder.append(this.summary_);

                builder.append(", ");
        builder.append("index=");
        builder.append(this.index_);

                builder.append(", ");
        builder.append("timestamped=");
        builder.append(this.timestamped_);

                builder.append(", ");
        builder.append("compressed=");
        builder.append(this.compressed_);

                
        builder.append("}");
		return builder.toString();
    }

    private us.ihmc.robotDataLogger.HandshakeFileType handshakeFileType_; 
    private StringBuilder handshake_; 
    private StringBuilder data_; 
    private StringBuilder summary_; 
    private StringBuilder index_; 
    private boolean timestamped_; 
    private boolean compressed_; 

}