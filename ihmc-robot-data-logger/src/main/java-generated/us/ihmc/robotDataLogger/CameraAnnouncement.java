package us.ihmc.robotDataLogger;
/**
* 
* Definition of the class "CameraAnnouncement" defined in Announcement.idl. 
*
* This file was automatically generated from Announcement.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Announcement.idl instead.
*
*/
public class CameraAnnouncement
{
    public CameraAnnouncement()
    {
        	name_ = new java.lang.StringBuilder(255); 
        	identifier_ = new java.lang.StringBuilder(255); 
        
        
    }

    public void set(CameraAnnouncement other)
    {
        	type_ = other.type_;
        	name_.setLength(0);
        	name_.append(other.name_);
        	identifier_.setLength(0);
        	identifier_.append(other.identifier_);

    }

    public void setType(us.ihmc.robotDataLogger.CameraType type)
    {
        type_ = type;
    }

    public us.ihmc.robotDataLogger.CameraType getType()
    {
        return type_;
    }

        
        public void setName(String name)
        {
        	name_.setLength(0);
        	name_.append(name);
        }
        
        public java.lang.String getNameAsString()
        {
        	return getName().toString();
        }

    public java.lang.StringBuilder getName()
    {
        return name_;
    }

        
        public void setIdentifier(String identifier)
        {
        	identifier_.setLength(0);
        	identifier_.append(identifier);
        }
        
        public java.lang.String getIdentifierAsString()
        {
        	return getIdentifier().toString();
        }

    public java.lang.StringBuilder getIdentifier()
    {
        return identifier_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof CameraAnnouncement)) return false;
        CameraAnnouncement otherMyClass = (CameraAnnouncement)other;
        boolean returnedValue = true;

        returnedValue &= this.type_ == otherMyClass.type_;

                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.identifier_, otherMyClass.identifier_);
                

        return returnedValue;
    }
    
     @Override
    public java.lang.String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("CameraAnnouncement {");
        builder.append("type=");
        builder.append(this.type_);

                builder.append(", ");
        builder.append("name=");
        builder.append(this.name_);

                builder.append(", ");
        builder.append("identifier=");
        builder.append(this.identifier_);

                
        builder.append("}");
		return builder.toString();
    }

    private us.ihmc.robotDataLogger.CameraType type_; 
    private java.lang.StringBuilder name_; 
    private java.lang.StringBuilder identifier_; 

}