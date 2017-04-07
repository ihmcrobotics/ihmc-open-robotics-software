package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "VariableChangeRequest" defined in VariableChangeRequest.idl. 
*
* This file was automatically generated from VariableChangeRequest.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit VariableChangeRequest.idl instead.
*
*/
public class VariableChangeRequest implements IDLStruct<VariableChangeRequest>
{
    public VariableChangeRequest()
    {
        
        
    }
    @Override
    public void set(VariableChangeRequest other)
    {
        	variableID_ = other.variableID_;
        	requestedValue_ = other.requestedValue_;

    }

    public void setVariableID(int variableID)
    {
        variableID_ = variableID;
    }

    public int getVariableID()
    {
        return variableID_;
    }

        
    public void setRequestedValue(double requestedValue)
    {
        requestedValue_ = requestedValue;
    }

    public double getRequestedValue()
    {
        return requestedValue_;
    }

        


	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(VariableChangeRequest data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(VariableChangeRequest data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    cdr.write_type_2(variableID_);

	    cdr.write_type_6(requestedValue_);
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	variableID_ = cdr.read_type_2();	

	    	requestedValue_ = cdr.read_type_6();	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_2("variableID", variableID_);
			    
			    ser.write_type_6("requestedValue", requestedValue_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			variableID_ = ser.read_type_2("variableID");	
	    	    
	    			requestedValue_ = ser.read_type_6("requestedValue");	
	    	    
	}

    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof VariableChangeRequest)) return false;
        VariableChangeRequest otherMyClass = (VariableChangeRequest)other;
        boolean returnedValue = true;

        returnedValue &= this.variableID_ == otherMyClass.variableID_;

                
        returnedValue &= this.requestedValue_ == otherMyClass.requestedValue_;

                

        return returnedValue;
    }
    
     @Override
    public String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("VariableChangeRequest {");
        builder.append("variableID=");
        builder.append(this.variableID_);

                builder.append(", ");
        builder.append("requestedValue=");
        builder.append(this.requestedValue_);

                
        builder.append("}");
		return builder.toString();
    }

    private int variableID_; 
    private double requestedValue_; 

}