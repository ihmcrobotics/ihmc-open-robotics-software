package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "LogProperties" defined in LogProperties.idl. 
*
* This file was automatically generated from LogProperties.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogProperties.idl instead.
*
*/
public class LogProperties implements IDLStruct<LogProperties>
{
    public LogProperties()
    {
        	version_ = new StringBuilder(255); 
        	name_ = new StringBuilder(255); 
        	variables_ = new us.ihmc.robotDataLogger.Variables();model_ = new us.ihmc.robotDataLogger.Model();timestamp_ = new StringBuilder(255); 
        	cameras_ = new IDLSequence.Object<us.ihmc.robotDataLogger.Camera> (255, us.ihmc.robotDataLogger.Camera.class, new us.ihmc.robotDataLogger.CameraPubSubType());

        	video_ = new us.ihmc.robotDataLogger.Video();
        
        
    }
    @Override
    public void set(LogProperties other)
    {
        	version_.setLength(0);
        	version_.append(other.version_);
        	name_.setLength(0);
        	name_.append(other.name_);
        	variables_.set(other.variables_);model_.set(other.model_);timestamp_.setLength(0);
        	timestamp_.append(other.timestamp_);
        	cameras_.set(other.cameras_);video_.set(other.video_);
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

        


	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += us.ihmc.robotDataLogger.Variables.getMaxCdrSerializedSize(current_alignment);
	    current_alignment += us.ihmc.robotDataLogger.Model.getMaxCdrSerializedSize(current_alignment);
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 255; ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.Camera.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += us.ihmc.robotDataLogger.Video.getMaxCdrSerializedSize(current_alignment);
	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(LogProperties data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(LogProperties data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getVersion().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += us.ihmc.robotDataLogger.Variables.getCdrSerializedSize(data.getVariables(), current_alignment);
	    current_alignment += us.ihmc.robotDataLogger.Model.getCdrSerializedSize(data.getModel(), current_alignment);
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getTimestamp().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getCameras().size(); ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.Camera.getCdrSerializedSize(data.getCameras().get(a), current_alignment);}

	    current_alignment += us.ihmc.robotDataLogger.Video.getCdrSerializedSize(data.getVideo(), current_alignment);
	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    if(version_.length() <= 255)
	    cdr.write_type_d(version_);else
	        throw new RuntimeException("version field exceeds the maximum length");

	    if(name_.length() <= 255)
	    cdr.write_type_d(name_);else
	        throw new RuntimeException("name field exceeds the maximum length");

	    cdr.write_type_a(variables_);

	    cdr.write_type_a(model_);

	    if(timestamp_.length() <= 255)
	    cdr.write_type_d(timestamp_);else
	        throw new RuntimeException("timestamp field exceeds the maximum length");

	    if(cameras_.size() <= 255)
	    cdr.write_type_e(cameras_);else
	        throw new RuntimeException("cameras field exceeds the maximum length");

	    cdr.write_type_a(video_);
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	cdr.read_type_d(version_);	

	    	cdr.read_type_d(name_);	

	    	cdr.read_type_a(variables_);	

	    	cdr.read_type_a(model_);	

	    	cdr.read_type_d(timestamp_);	

	    	cdr.read_type_e(cameras_);	

	    	cdr.read_type_a(video_);	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_d("version", version_);
			    
			    ser.write_type_d("name", name_);
			    
			    ser.write_type_a("variables", variables_);
			    
			    ser.write_type_a("model", model_);
			    
			    ser.write_type_d("timestamp", timestamp_);
			    
			    ser.write_type_e("cameras", cameras_);
			    
			    ser.write_type_a("video", video_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			ser.read_type_d("version", version_);	
	    	    
	    			ser.read_type_d("name", name_);	
	    	    
	    			ser.read_type_a("variables", variables_);	
	    	    
	    			ser.read_type_a("model", model_);	
	    	    
	    			ser.read_type_d("timestamp", timestamp_);	
	    	    
	    			ser.read_type_e("cameras", cameras_);	
	    	    
	    			ser.read_type_a("video", video_);	
	    	    
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