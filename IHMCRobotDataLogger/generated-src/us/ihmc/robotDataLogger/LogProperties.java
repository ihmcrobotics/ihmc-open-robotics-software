package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import java.util.Arrays;

/**
* 
* Definition of the class "LogProperties" defined in LogProperties.idl. 
*
* This file was automatically generated from LogProperties.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogProperties.idl instead.
*
*/
public class LogProperties
{
    public LogProperties()
    {
        	version_ = new StringBuilder(255); 
        	name_ = new StringBuilder(255); 
        	variables_ = new us.ihmc.robotDataLogger.Variables();model_ = new us.ihmc.robotDataLogger.Model();timestamp_ = new StringBuilder(255); 
        	cameras_ = new IDLSequence.Object<us.ihmc.robotDataLogger.Camera> (255, us.ihmc.robotDataLogger.Camera.class, new us.ihmc.robotDataLogger.CameraPubSubType());

        	video_ = new us.ihmc.robotDataLogger.Video();
        
        
    }

    public void set(LogProperties other)
    {
        	version_.setLength(0);
        	version_.append(other.version_);
        	name_.setLength(0);
        	name_.append(other.name_);
           	us.ihmc.robotDataLogger.VariablesPubSubType.staticCopy(variables_, other.variables_);us.ihmc.robotDataLogger.ModelPubSubType.staticCopy(model_, other.model_);timestamp_.setLength(0);
        	timestamp_.append(other.timestamp_);
            cameras_.set(other.cameras_);	us.ihmc.robotDataLogger.VideoPubSubType.staticCopy(video_, other.video_);
    }

        public void setVersion(String version)
        {
        	version_.setLength(0);
        	version_.append(version);
        }
        
        public String getVersionAsString()
        {
        	return getVersion().toString();
        }

    public StringBuilder getVersion()
    {
        return version_;
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

        

    public us.ihmc.robotDataLogger.Variables getVariables()
    {
        return variables_;
    }

        

    public us.ihmc.robotDataLogger.Model getModel()
    {
        return model_;
    }

        
        public void setTimestamp(String timestamp)
        {
        	timestamp_.setLength(0);
        	timestamp_.append(timestamp);
        }
        
        public String getTimestampAsString()
        {
        	return getTimestamp().toString();
        }

    public StringBuilder getTimestamp()
    {
        return timestamp_;
    }

        

    public IDLSequence.Object<us.ihmc.robotDataLogger.Camera>  getCameras()
    {
        return cameras_;
    }

        

    public us.ihmc.robotDataLogger.Video getVideo()
    {
        return video_;
    }

        




    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof LogProperties)) return false;
        LogProperties otherMyClass = (LogProperties)other;
        boolean returnedValue = true;

        returnedValue &= us.ihmc.idl.IDLTools.equals(this.version_, otherMyClass.version_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_);
                
        returnedValue &= this.variables_.equals(otherMyClass.variables_);
                
        returnedValue &= this.model_.equals(otherMyClass.model_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.timestamp_, otherMyClass.timestamp_);
                
        returnedValue &= this.cameras_.equals(otherMyClass.cameras_);
                
        returnedValue &= this.video_.equals(otherMyClass.video_);
                

        return returnedValue;
    }
    
     @Override
    public String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("LogProperties {");
        builder.append("version=");
        builder.append(this.version_);

                builder.append(", ");
        builder.append("name=");
        builder.append(this.name_);

                builder.append(", ");
        builder.append("variables=");
        builder.append(this.variables_);

                builder.append(", ");
        builder.append("model=");
        builder.append(this.model_);

                builder.append(", ");
        builder.append("timestamp=");
        builder.append(this.timestamp_);

                builder.append(", ");
        builder.append("cameras=");
        builder.append(this.cameras_);

                builder.append(", ");
        builder.append("video=");
        builder.append(this.video_);

                
        builder.append("}");
		return builder.toString();
    }

    private StringBuilder version_; 
    private StringBuilder name_; 
    private us.ihmc.robotDataLogger.Variables variables_; 
    private us.ihmc.robotDataLogger.Model model_; 
    private StringBuilder timestamp_; 
    private IDLSequence.Object<us.ihmc.robotDataLogger.Camera>  cameras_; 
    private us.ihmc.robotDataLogger.Video video_; 

}