package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import java.util.Arrays;

/**
* 
* Definition of the class "LogData" defined in LogData.idl. 
*
* This file was automatically generated from LogData.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogData.idl instead.
*
*/
public class LogData
{
    public LogData()
    {
        	data_ = new IDLSequence.Byte (100, "type_9");
        
        
    }

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