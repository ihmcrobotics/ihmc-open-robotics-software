package us.ihmc.robotDataLogger;
/**
* 
* Definition of the class "Timestamp" defined in Timestamp.idl. 
*
* This file was automatically generated from Timestamp.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Timestamp.idl instead.
*
*/
public class Timestamp
{
    public Timestamp()
    {
        
        
    }

    public void set(Timestamp other)
    {
        	timestamp_ = other.timestamp_;

    }

    public void setTimestamp(long timestamp)
    {
        timestamp_ = timestamp;
    }

    public long getTimestamp()
    {
        return timestamp_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof Timestamp)) return false;
        Timestamp otherMyClass = (Timestamp)other;
        boolean returnedValue = true;

        returnedValue &= this.timestamp_ == otherMyClass.timestamp_;

                

        return returnedValue;
    }
    
     @Override
    public java.lang.String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("Timestamp {");
        builder.append("timestamp=");
        builder.append(this.timestamp_);

                
        builder.append("}");
		return builder.toString();
    }

    private long timestamp_; 

}