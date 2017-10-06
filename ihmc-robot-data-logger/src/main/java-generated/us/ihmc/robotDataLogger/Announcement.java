package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import java.util.Arrays;

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
        	identifier_ = new StringBuilder(255); 
        	name_ = new StringBuilder(255); 
        	hostName_ = new StringBuilder(255); 
        	cameras_ = new IDLSequence.Object<us.ihmc.robotDataLogger.CameraAnnouncement> (127, us.ihmc.robotDataLogger.CameraAnnouncement.class, new us.ihmc.robotDataLogger.CameraAnnouncementPubSubType());

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
            cameras_.set(other.cameras_);	us.ihmc.robotDataLogger.ModelFileDescriptionPubSubType.staticCopy(other.modelFileDescription_, modelFileDescription_);log_ = other.log_;

    }

        public void setIdentifier(String identifier)
        {
        	identifier_.setLength(0);
        	identifier_.append(identifier);
        }
        
        public String getIdentifierAsString()
        {
        	return getIdentifier().toString();
        }

    public StringBuilder getIdentifier()
    {
        return identifier_;
    }

        
        public void setName(String name)
        {
        	name_.setLength(0);
        	name_.append(name);
        }
        
        public String getNameAsString()
        {
        	return getName().toString();
        }

    public StringBuilder getName()
    {
        return name_;
    }

        
        public void setHostName(String hostName)
        {
        	hostName_.setLength(0);
        	hostName_.append(hostName);
        }
        
        public String getHostNameAsString()
        {
        	return getHostName().toString();
        }

    public StringBuilder getHostName()
    {
        return hostName_;
    }

        

    public IDLSequence.Object<us.ihmc.robotDataLogger.CameraAnnouncement>  getCameras()
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
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof Announcement)) return false;
        Announcement otherMyClass = (Announcement)other;
        boolean returnedValue = true;

        returnedValue &= us.ihmc.idl.IDLTools.equals(this.identifier_, otherMyClass.identifier_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.hostName_, otherMyClass.hostName_);
                
        returnedValue &= this.cameras_.equals(otherMyClass.cameras_);
                
        returnedValue &= this.modelFileDescription_.equals(otherMyClass.modelFileDescription_);
                
        returnedValue &= this.log_ == otherMyClass.log_;

                

        return returnedValue;
    }
    
     @Override
    public String toString()
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

    private StringBuilder identifier_; 
    private StringBuilder name_; 
    private StringBuilder hostName_; 
    private IDLSequence.Object<us.ihmc.robotDataLogger.CameraAnnouncement>  cameras_; 
    private us.ihmc.robotDataLogger.ModelFileDescription modelFileDescription_; 
    private boolean log_; 

}