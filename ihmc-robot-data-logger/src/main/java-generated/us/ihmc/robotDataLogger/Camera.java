package us.ihmc.robotDataLogger;
/**
* 
* Definition of the class "Camera" defined in LogProperties.idl. 
*
* This file was automatically generated from LogProperties.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogProperties.idl instead.
*
*/
public class Camera
{
    public Camera()
    {
        	name_ = new java.lang.StringBuilder(255); 
        	videoFile_ = new java.lang.StringBuilder(255); 
        	timestampFile_ = new java.lang.StringBuilder(255); 
        
        
    }

    public void set(Camera other)
    {
        	name_.setLength(0);
        	name_.append(other.name_);
        	interlaced_ = other.interlaced_;
        	videoFile_.setLength(0);
        	videoFile_.append(other.videoFile_);
        	timestampFile_.setLength(0);
        	timestampFile_.append(other.timestampFile_);

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

        
    public void setInterlaced(boolean interlaced)
    {
        interlaced_ = interlaced;
    }

    public boolean getInterlaced()
    {
        return interlaced_;
    }

        
        public void setVideoFile(String videoFile)
        {
        	videoFile_.setLength(0);
        	videoFile_.append(videoFile);
        }
        
        public java.lang.String getVideoFileAsString()
        {
        	return getVideoFile().toString();
        }

    public java.lang.StringBuilder getVideoFile()
    {
        return videoFile_;
    }

        
        public void setTimestampFile(String timestampFile)
        {
        	timestampFile_.setLength(0);
        	timestampFile_.append(timestampFile);
        }
        
        public java.lang.String getTimestampFileAsString()
        {
        	return getTimestampFile().toString();
        }

    public java.lang.StringBuilder getTimestampFile()
    {
        return timestampFile_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof Camera)) return false;
        Camera otherMyClass = (Camera)other;
        boolean returnedValue = true;

        returnedValue &= us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_);
                
        returnedValue &= this.interlaced_ == otherMyClass.interlaced_;

                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.videoFile_, otherMyClass.videoFile_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.timestampFile_, otherMyClass.timestampFile_);
                

        return returnedValue;
    }
    
     @Override
    public java.lang.String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("Camera {");
        builder.append("name=");
        builder.append(this.name_);

                builder.append(", ");
        builder.append("interlaced=");
        builder.append(this.interlaced_);

                builder.append(", ");
        builder.append("videoFile=");
        builder.append(this.videoFile_);

                builder.append(", ");
        builder.append("timestampFile=");
        builder.append(this.timestampFile_);

                
        builder.append("}");
		return builder.toString();
    }

    private java.lang.StringBuilder name_; 
    private boolean interlaced_; 
    private java.lang.StringBuilder videoFile_; 
    private java.lang.StringBuilder timestampFile_; 

}