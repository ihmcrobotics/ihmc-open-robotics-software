package us.ihmc.robotDataLogger;
/**
* 
* Definition of the class "Announcement" defined in Announcement.idl. 
*
* This file was automatically generated from Announcement.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Announcement.idl instead.
*
*/
public class Announcement
{
    public Announcement()
    {
        	identifier_ = new java.lang.StringBuilder(255); 
        	name_ = new java.lang.StringBuilder(255); 
        	hostName_ = new java.lang.StringBuilder(255); 
        	reconnectKey_ = new java.lang.StringBuilder(255); 
        	cameras_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.CameraAnnouncement> (127, us.ihmc.robotDataLogger.CameraAnnouncement.class, new us.ihmc.robotDataLogger.CameraAnnouncementPubSubType());

        	modelFileDescription_ = new us.ihmc.robotDataLogger.ModelFileDescription();
        
        
    }

    public void set(Announcement other)
    {
        	identifier_.setLength(0);
        	identifier_.append(other.identifier_);
        	name_.setLength(0);
        	name_.append(other.name_);
        	hostName_.setLength(0);
        	hostName_.append(other.hostName_);
        	reconnectKey_.setLength(0);
        	reconnectKey_.append(other.reconnectKey_);
            cameras_.set(other.cameras_);	us.ihmc.robotDataLogger.ModelFileDescriptionPubSubType.staticCopy(other.modelFileDescription_, modelFileDescription_);log_ = other.log_;

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

        
        public void setHostName(String hostName)
        {
        	hostName_.setLength(0);
        	hostName_.append(hostName);
        }
        
        public java.lang.String getHostNameAsString()
        {
        	return getHostName().toString();
        }

    public java.lang.StringBuilder getHostName()
    {
        return hostName_;
    }

        
        public void setReconnectKey(String reconnectKey)
        {
        	reconnectKey_.setLength(0);
        	reconnectKey_.append(reconnectKey);
        }
        
        public java.lang.String getReconnectKeyAsString()
        {
        	return getReconnectKey().toString();
        }

    public java.lang.StringBuilder getReconnectKey()
    {
        return reconnectKey_;
    }

        

    public us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.CameraAnnouncement>  getCameras()
    {
        return cameras_;
    }

        

    public us.ihmc.robotDataLogger.ModelFileDescription getModelFileDescription()
    {
        return modelFileDescription_;
    }

        
    public void setLog(boolean log)
    {
        log_ = log;
    }

    public boolean getLog()
    {
        return log_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof Announcement)) return false;
        Announcement otherMyClass = (Announcement)other;
        boolean returnedValue = true;

        returnedValue &= us.ihmc.idl.IDLTools.equals(this.identifier_, otherMyClass.identifier_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.hostName_, otherMyClass.hostName_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.reconnectKey_, otherMyClass.reconnectKey_);
                
        returnedValue &= this.cameras_.equals(otherMyClass.cameras_);
                
        returnedValue &= this.modelFileDescription_.equals(otherMyClass.modelFileDescription_);
                
        returnedValue &= this.log_ == otherMyClass.log_;

                

        return returnedValue;
    }
    
     @Override
    public java.lang.String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("Announcement {");
        builder.append("identifier=");
        builder.append(this.identifier_);

                builder.append(", ");
        builder.append("name=");
        builder.append(this.name_);

                builder.append(", ");
        builder.append("hostName=");
        builder.append(this.hostName_);

                builder.append(", ");
        builder.append("reconnectKey=");
        builder.append(this.reconnectKey_);

                builder.append(", ");
        builder.append("cameras=");
        builder.append(this.cameras_);

                builder.append(", ");
        builder.append("modelFileDescription=");
        builder.append(this.modelFileDescription_);

                builder.append(", ");
        builder.append("log=");
        builder.append(this.log_);

                
        builder.append("}");
		return builder.toString();
    }

    private java.lang.StringBuilder identifier_; 
    private java.lang.StringBuilder name_; 
    private java.lang.StringBuilder hostName_; 
    private java.lang.StringBuilder reconnectKey_; 
    private us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.CameraAnnouncement>  cameras_; 
    private us.ihmc.robotDataLogger.ModelFileDescription modelFileDescription_; 
    private boolean log_; 

}