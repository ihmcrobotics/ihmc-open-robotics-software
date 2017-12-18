package us.ihmc.robotDataLogger;
/**
* 
* Definition of the class "VariableChangeRequest" defined in VariableChangeRequest.idl. 
*
* This file was automatically generated from VariableChangeRequest.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit VariableChangeRequest.idl instead.
*
*/
public class VariableChangeRequest
{
    public VariableChangeRequest()
    {
        
        
    }

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

        




    @Override
    public boolean equals(java.lang.Object other)
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
    public java.lang.String toString()
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