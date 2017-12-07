package us.ihmc.robotDataLogger;
/**
* 
* Definition of the class "Variables" defined in LogProperties.idl. 
*
* This file was automatically generated from LogProperties.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogProperties.idl instead.
*
*/
public class Variables
{
    public Variables()
    {
        	handshake_ = new java.lang.StringBuilder(255); 
        	data_ = new java.lang.StringBuilder(255); 
        	summary_ = new java.lang.StringBuilder(255); 
        	index_ = new java.lang.StringBuilder(255); 
        
        
    }

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
        
        public java.lang.String getHandshakeAsString()
        {
        	return getHandshake().toString();
        }

    public java.lang.StringBuilder getHandshake()
    {
        return handshake_;
    }

        
        public void setData(String data)
        {
        	data_.setLength(0);
        	data_.append(data);
        }
        
        public java.lang.String getDataAsString()
        {
        	return getData().toString();
        }

    public java.lang.StringBuilder getData()
    {
        return data_;
    }

        
        public void setSummary(String summary)
        {
        	summary_.setLength(0);
        	summary_.append(summary);
        }
        
        public java.lang.String getSummaryAsString()
        {
        	return getSummary().toString();
        }

    public java.lang.StringBuilder getSummary()
    {
        return summary_;
    }

        
        public void setIndex(String index)
        {
        	index_.setLength(0);
        	index_.append(index);
        }
        
        public java.lang.String getIndexAsString()
        {
        	return getIndex().toString();
        }

    public java.lang.StringBuilder getIndex()
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

        




    @Override
    public boolean equals(java.lang.Object other)
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
    public java.lang.String toString()
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
    private java.lang.StringBuilder handshake_; 
    private java.lang.StringBuilder data_; 
    private java.lang.StringBuilder summary_; 
    private java.lang.StringBuilder index_; 
    private boolean timestamped_; 
    private boolean compressed_; 

}