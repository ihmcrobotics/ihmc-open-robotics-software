package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "CameraAnnouncement" defined in Announcement.idl. 
*
* This file was automatically generated from Announcement.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Announcement.idl instead.
*
*/
public class CameraAnnouncement implements IDLStruct<CameraAnnouncement>
{
    public CameraAnnouncement()
    {
        	name_ = new StringBuilder(255); 
        	identifier_ = new StringBuilder(255); 
        
        
    }
    @Override
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
        
        public String getNameAsString()
        {
        	return getName().toString();
        }

    public StringBuilder getName()
    {
        return name_;
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

        


	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(CameraAnnouncement data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(CameraAnnouncement data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getIdentifier().length() + 1;

	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    cdr.write_type_c(type_.ordinal());


	    if(name_.length() <= 255)
	    cdr.write_type_d(name_);else
	        throw new RuntimeException("name field exceeds the maximum length");

	    if(identifier_.length() <= 255)
	    cdr.write_type_d(identifier_);else
	        throw new RuntimeException("identifier field exceeds the maximum length");
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	type_ = us.ihmc.robotDataLogger.CameraType.values[cdr.read_type_c()];
	    	

	    	cdr.read_type_d(name_);	

	    	cdr.read_type_d(identifier_);	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_c("type", type_);
			    
			    ser.write_type_d("name", name_);
			    
			    ser.write_type_d("identifier", identifier_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			type_ = (us.ihmc.robotDataLogger.CameraType)ser.read_type_c("type", us.ihmc.robotDataLogger.CameraType.class);
	    	
	    	    
	    			ser.read_type_d("name", name_);	
	    	    
	    			ser.read_type_d("identifier", identifier_);	
	    	    
	}

    @Override
    public boolean equals(Object other)
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
    public String toString()
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
    private StringBuilder name_; 
    private StringBuilder identifier_; 

}