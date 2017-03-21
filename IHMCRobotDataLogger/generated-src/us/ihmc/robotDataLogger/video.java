package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "video" defined in LogProperties.idl. 
*
* This file was automatically generated from LogProperties.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogProperties.idl instead.
*
*/
public class video implements IDLStruct<video>
{
    public video()
    {
        
        
    }
    @Override
    public void set(video other)
    {
        	hasTimebase_ = other.hasTimebase_;

    }

    public void setHasTimebase(boolean hasTimebase)
    {
        hasTimebase_ = hasTimebase;
    }

    public boolean getHasTimebase()
    {
        return hasTimebase_;
    }

        


	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(video data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(video data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    cdr.write_type_7(hasTimebase_);
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	hasTimebase_ = cdr.read_type_7();	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_7("hasTimebase", hasTimebase_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			hasTimebase_ = ser.read_type_7("hasTimebase");	
	    	    
	}

    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof video)) return false;
        video otherMyClass = (video)other;
        boolean returnedValue = true;

        returnedValue &= this.hasTimebase_ == otherMyClass.hasTimebase_;

                

        return returnedValue;
    }
    
     @Override
    public String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("video {");
        builder.append("hasTimebase=");
        builder.append(this.hasTimebase_);

                
        builder.append("}");
		return builder.toString();
    }

    private boolean hasTimebase_; 

}