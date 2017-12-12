package us.ihmc.robotDataLogger;
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
        	data_ = new us.ihmc.idl.IDLSequence.Byte (100, "type_9");
        	jointStates_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");
        
        
    }

    public void set(LogData other)
    {
        	uid_ = other.uid_;
        	timestamp_ = other.timestamp_;
        	transmitTime_ = other.transmitTime_;
        	type_ = other.type_;
        	registry_ = other.registry_;
        	offset_ = other.offset_;
        	numberOfVariables_ = other.numberOfVariables_;
            data_.set(other.data_);	jointStates_.set(other.jointStates_);	
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

        
    public void setTransmitTime(long transmitTime)
    {
        transmitTime_ = transmitTime;
    }

    public long getTransmitTime()
    {
        return transmitTime_;
    }

        
    public void setType(us.ihmc.robotDataLogger.LogDataType type)
    {
        type_ = type;
    }

    public us.ihmc.robotDataLogger.LogDataType getType()
    {
        return type_;
    }

        
    public void setRegistry(int registry)
    {
        registry_ = registry;
    }

    public int getRegistry()
    {
        return registry_;
    }

        
    public void setOffset(int offset)
    {
        offset_ = offset;
    }

    public int getOffset()
    {
        return offset_;
    }

        
    public void setNumberOfVariables(int numberOfVariables)
    {
        numberOfVariables_ = numberOfVariables;
    }

    public int getNumberOfVariables()
    {
        return numberOfVariables_;
    }

        

    public us.ihmc.idl.IDLSequence.Byte  getData()
    {
        return data_;
    }

        

    public us.ihmc.idl.IDLSequence.Double  getJointStates()
    {
        return jointStates_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof LogData)) return false;
        LogData otherMyClass = (LogData)other;
        boolean returnedValue = true;

        returnedValue &= this.uid_ == otherMyClass.uid_;

                
        returnedValue &= this.timestamp_ == otherMyClass.timestamp_;

                
        returnedValue &= this.transmitTime_ == otherMyClass.transmitTime_;

                
        returnedValue &= this.type_ == otherMyClass.type_;

                
        returnedValue &= this.registry_ == otherMyClass.registry_;

                
        returnedValue &= this.offset_ == otherMyClass.offset_;

                
        returnedValue &= this.numberOfVariables_ == otherMyClass.numberOfVariables_;

                
        returnedValue &= this.data_.equals(otherMyClass.data_);
                
        returnedValue &= this.jointStates_.equals(otherMyClass.jointStates_);
                

        return returnedValue;
    }
    
     @Override
    public java.lang.String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("LogData {");
        builder.append("uid=");
        builder.append(this.uid_);

                builder.append(", ");
        builder.append("timestamp=");
        builder.append(this.timestamp_);

                builder.append(", ");
        builder.append("transmitTime=");
        builder.append(this.transmitTime_);

                builder.append(", ");
        builder.append("type=");
        builder.append(this.type_);

                builder.append(", ");
        builder.append("registry=");
        builder.append(this.registry_);

                builder.append(", ");
        builder.append("offset=");
        builder.append(this.offset_);

                builder.append(", ");
        builder.append("numberOfVariables=");
        builder.append(this.numberOfVariables_);

                builder.append(", ");
        builder.append("data=");
        builder.append(this.data_);

                builder.append(", ");
        builder.append("jointStates=");
        builder.append(this.jointStates_);

                
        builder.append("}");
		return builder.toString();
    }

    private long uid_; 
    private long timestamp_; 
    private long transmitTime_; 
    private us.ihmc.robotDataLogger.LogDataType type_; 
    private int registry_; 
    private int offset_; 
    private int numberOfVariables_; 
    private us.ihmc.idl.IDLSequence.Byte  data_; 
    private us.ihmc.idl.IDLSequence.Double  jointStates_; 

}