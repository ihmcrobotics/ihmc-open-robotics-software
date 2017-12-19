package us.ihmc.robotDataLogger;
/**
* 
* Definition of the class "LoggerConfiguration" defined in LoggerConfiguration.idl. 
*
* This file was automatically generated from LoggerConfiguration.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LoggerConfiguration.idl instead.
*
*/
public class LoggerConfiguration
{
    public LoggerConfiguration()
    {
        	camerasToCapture_ = new java.lang.StringBuilder(255); 
        
        
    }

    public void set(LoggerConfiguration other)
    {
        	camerasToCapture_.setLength(0);
        	camerasToCapture_.append(other.camerasToCapture_);
        	publicBroadcast_ = other.publicBroadcast_;

    }

        public void setCamerasToCapture(String camerasToCapture)
        {
        	camerasToCapture_.setLength(0);
        	camerasToCapture_.append(camerasToCapture);
        }
        
        public java.lang.String getCamerasToCaptureAsString()
        {
        	return getCamerasToCapture().toString();
        }

    public java.lang.StringBuilder getCamerasToCapture()
    {
        return camerasToCapture_;
    }

        
    public void setPublicBroadcast(boolean publicBroadcast)
    {
        publicBroadcast_ = publicBroadcast;
    }

    public boolean getPublicBroadcast()
    {
        return publicBroadcast_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof LoggerConfiguration)) return false;
        LoggerConfiguration otherMyClass = (LoggerConfiguration)other;
        boolean returnedValue = true;

        returnedValue &= us.ihmc.idl.IDLTools.equals(this.camerasToCapture_, otherMyClass.camerasToCapture_);
                
        returnedValue &= this.publicBroadcast_ == otherMyClass.publicBroadcast_;

                

        return returnedValue;
    }
    
     @Override
    public java.lang.String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("LoggerConfiguration {");
        builder.append("camerasToCapture=");
        builder.append(this.camerasToCapture_);

                builder.append(", ");
        builder.append("publicBroadcast=");
        builder.append(this.publicBroadcast_);

                
        builder.append("}");
		return builder.toString();
    }

    private java.lang.StringBuilder camerasToCapture_; 
    private boolean publicBroadcast_; 

}