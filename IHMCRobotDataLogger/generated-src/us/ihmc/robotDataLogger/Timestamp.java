package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "Timestamp" defined in Timestamp.idl. 
*
* This file was automatically generated from Timestamp.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Timestamp.idl instead.
*
*/
public class Timestamp implements IDLStruct<Timestamp>
{
    public Timestamp()
    {
        
        
    }
    @Override
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

        


	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(Timestamp data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(Timestamp data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    cdr.write_type_11(timestamp_);
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	timestamp_ = cdr.read_type_11();	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_11("timestamp", timestamp_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			timestamp_ = ser.read_type_11("timestamp");	
	    	    
	}

    @Override
    public boolean equals(Object other)
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
    public String toString()
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