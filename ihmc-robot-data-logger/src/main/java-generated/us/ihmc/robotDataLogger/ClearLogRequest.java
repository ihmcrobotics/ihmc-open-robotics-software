package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import java.util.Arrays;

/**
* 
* Definition of the class "ClearLogRequest" defined in ClearLogRequest.idl. 
*
* This file was automatically generated from ClearLogRequest.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ClearLogRequest.idl instead.
*
*/
public class ClearLogRequest
{
    public ClearLogRequest()
    {
        	guid_ = new StringBuilder(255); 
        
        
    }

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