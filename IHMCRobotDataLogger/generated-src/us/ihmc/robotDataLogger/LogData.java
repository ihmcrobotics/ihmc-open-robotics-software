package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "LogData" defined in LogData.idl. 
*
* This file was automatically generated from LogData.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogData.idl instead.
*
*/
public class LogData implements IDLStruct<LogData>
{
    public LogData()
    {
        	data_ = new IDLSequence.Byte (100, "type_9");
        
        
    }
    @Override
    public void set(LogData other)
    {
        	uid_ = other.uid_;
        	timestamp_ = other.timestamp_;
        	data_.set(other.data_);
    }

    public void setUid(long uid)
    {
        uid_ = uid;
    }

    public long getUid()
    {
        return uid_;
    }

        
    public void setTimestamp(long timestamp)
    {
        timestamp_ = timestamp;
    }

    public long getTimestamp()
    {
        return timestamp_;
    }

        

    public IDLSequence.Byte  getData()
    {
        return data_;
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


	public final static int getCdrSerializedSize(LogData data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(LogData data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    current_alignment += (data.getData().size() * 1) + CDR.alignment(current_alignment, 1);


	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    cdr.write_type_11(uid_);

	    cdr.write_type_11(timestamp_);

	    if(data_.size() <= 100)
	    cdr.write_type_e(data_);else
	        throw new RuntimeException("data field exceeds the maximum length");
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	uid_ = cdr.read_type_11();	

	    	timestamp_ = cdr.read_type_11();	

	    	cdr.read_type_e(data_);	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_11("uid", uid_);
			    
			    ser.write_type_11("timestamp", timestamp_);
			    
			    ser.write_type_e("data", data_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			uid_ = ser.read_type_11("uid");	
	    	    
	    			timestamp_ = ser.read_type_11("timestamp");	
	    	    
	    			ser.read_type_e("data", data_);	
	    	    
	}

    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof LogData)) return false;
        LogData otherMyClass = (LogData)other;
        boolean returnedValue = true;

        returnedValue &= this.uid_ == otherMyClass.uid_;

                
        returnedValue &= this.timestamp_ == otherMyClass.timestamp_;

                
        returnedValue &= this.data_.equals(otherMyClass.data_);
                

        return returnedValue;
    }
    
     @Override
    public String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("LogData {");
        builder.append("uid=");
        builder.append(this.uid_);

                builder.append(", ");
        builder.append("timestamp=");
        builder.append(this.timestamp_);

                builder.append(", ");
        builder.append("data=");
        builder.append(this.data_);

                
        builder.append("}");
		return builder.toString();
    }

    private long uid_; 
    private long timestamp_; 
    private IDLSequence.Byte  data_; 

}