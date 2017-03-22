package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "ClearLogRequest" defined in ClearLogRequest.idl. 
*
* This file was automatically generated from ClearLogRequest.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ClearLogRequest.idl instead.
*
*/
public class ClearLogRequest implements IDLStruct<ClearLogRequest>
{
    public ClearLogRequest()
    {
        	guid_ = new StringBuilder(255); 
        
        
    }
    @Override
    public void set(ClearLogRequest other)
    {
        	guid_.setLength(0);
        	guid_.append(other.guid_);
    }

        public void setGuid(String guid)
        {
        	guid_.setLength(0);
        	guid_.append(guid);
        }
        
        public String getGuidAsString()
        {
        	return getGuid().toString();
        }

    public StringBuilder getGuid()
    {
        return guid_;
    }

        


	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;
	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(ClearLogRequest data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(ClearLogRequest data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getGuid().length() + 1;
	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    if(guid_.length() <= 255)
	    cdr.write_type_d(guid_);else
	        throw new RuntimeException("guid field exceeds the maximum length");
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	cdr.read_type_d(guid_);	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_d("guid", guid_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			ser.read_type_d("guid", guid_);	
	    	    
	}

    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof ClearLogRequest)) return false;
        ClearLogRequest otherMyClass = (ClearLogRequest)other;
        boolean returnedValue = true;

        returnedValue &= us.ihmc.idl.IDLTools.equals(this.guid_, otherMyClass.guid_);
                

        return returnedValue;
    }
    
     @Override
    public String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("ClearLogRequest {");
        builder.append("guid=");
        builder.append(this.guid_);

                
        builder.append("}");
		return builder.toString();
    }

    private StringBuilder guid_; 

}