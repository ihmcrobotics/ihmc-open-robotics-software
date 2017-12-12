package us.ihmc.robotDataLogger;
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
        	guid_ = new java.lang.StringBuilder(255); 
        
        
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
        
        public java.lang.String getGuidAsString()
        {
        	return getGuid().toString();
        }

    public java.lang.StringBuilder getGuid()
    {
        return guid_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
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
    public java.lang.String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("ClearLogRequest {");
        builder.append("guid=");
        builder.append(this.guid_);

                
        builder.append("}");
		return builder.toString();
    }

    private java.lang.StringBuilder guid_; 

}