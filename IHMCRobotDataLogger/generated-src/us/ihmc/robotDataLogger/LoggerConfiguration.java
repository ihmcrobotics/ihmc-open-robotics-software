package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import java.util.Arrays;

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
        	camerasToCapture_ = new StringBuilder(255); 
        	loggerNetwork_ = new StringBuilder(255); 
        
        
    }

    public void set(LoggerConfiguration other)
    {
        	camerasToCapture_.setLength(0);
        	camerasToCapture_.append(other.camerasToCapture_);
        	loggerNetwork_.setLength(0);
        	loggerNetwork_.append(other.loggerNetwork_);
    }

        public void setCamerasToCapture(String camerasToCapture)
        {
        	camerasToCapture_.setLength(0);
        	camerasToCapture_.append(camerasToCapture);
        }
        
        public String getCamerasToCaptureAsString()
        {
        	return getCamerasToCapture().toString();
        }

    public StringBuilder getCamerasToCapture()
    {
        return camerasToCapture_;
    }

        
        public void setLoggerNetwork(String loggerNetwork)
        {
        	loggerNetwork_.setLength(0);
        	loggerNetwork_.append(loggerNetwork);
        }
        
        public String getLoggerNetworkAsString()
        {
        	return getLoggerNetwork().toString();
        }

    public StringBuilder getLoggerNetwork()
    {
        return loggerNetwork_;
    }

        




    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof LoggerConfiguration)) return false;
        LoggerConfiguration otherMyClass = (LoggerConfiguration)other;
        boolean returnedValue = true;

        returnedValue &= us.ihmc.idl.IDLTools.equals(this.camerasToCapture_, otherMyClass.camerasToCapture_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.loggerNetwork_, otherMyClass.loggerNetwork_);
                

        return returnedValue;
    }
    
     @Override
    public String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("LoggerConfiguration {");
        builder.append("camerasToCapture=");
        builder.append(this.camerasToCapture_);

                builder.append(", ");
        builder.append("loggerNetwork=");
        builder.append(this.loggerNetwork_);

                
        builder.append("}");
		return builder.toString();
    }

    private StringBuilder camerasToCapture_; 
    private StringBuilder loggerNetwork_; 

}