package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "Camera" defined in LogProperties.idl. 
*
* This file was automatically generated from LogProperties.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogProperties.idl instead.
*
*/
public class Camera implements IDLStruct<Camera>
{
    public Camera()
    {
        	name_ = new StringBuilder(255); 
        	videoFile_ = new StringBuilder(255); 
        	timestampFile_ = new StringBuilder(255); 
        
        
    }
    @Override
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
        
        public String getNameAsString()
        {
        	return getName().toString();
        }

    public StringBuilder getName()
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
        
        public String getVideoFileAsString()
        {
        	return getVideoFile().toString();
        }

    public StringBuilder getVideoFile()
    {
        return videoFile_;
    }

        
        public void setTimestampFile(String timestampFile)
        {
        	timestampFile_.setLength(0);
        	timestampFile_.append(timestampFile);
        }
        
        public String getTimestampFileAsString()
        {
        	return getTimestampFile().toString();
        }

    public StringBuilder getTimestampFile()
    {
        return timestampFile_;
    }

        


	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(Camera data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(Camera data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getVideoFile().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getTimestampFile().length() + 1;

	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    if(name_.length() <= 255)
	    cdr.write_type_d(name_);else
	        throw new RuntimeException("name field exceeds the maximum length");

	    cdr.write_type_7(interlaced_);

	    if(videoFile_.length() <= 255)
	    cdr.write_type_d(videoFile_);else
	        throw new RuntimeException("videoFile field exceeds the maximum length");

	    if(timestampFile_.length() <= 255)
	    cdr.write_type_d(timestampFile_);else
	        throw new RuntimeException("timestampFile field exceeds the maximum length");
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	cdr.read_type_d(name_);	

	    	interlaced_ = cdr.read_type_7();	

	    	cdr.read_type_d(videoFile_);	

	    	cdr.read_type_d(timestampFile_);	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_d("name", name_);
			    
			    ser.write_type_7("interlaced", interlaced_);
			    
			    ser.write_type_d("videoFile", videoFile_);
			    
			    ser.write_type_d("timestampFile", timestampFile_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			ser.read_type_d("name", name_);	
	    	    
	    			interlaced_ = ser.read_type_7("interlaced");	
	    	    
	    			ser.read_type_d("videoFile", videoFile_);	
	    	    
	    			ser.read_type_d("timestampFile", timestampFile_);	
	    	    
	}

    @Override
    public boolean equals(Object other)
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
    public String toString()
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

    private StringBuilder name_; 
    private boolean interlaced_; 
    private StringBuilder videoFile_; 
    private StringBuilder timestampFile_; 

}