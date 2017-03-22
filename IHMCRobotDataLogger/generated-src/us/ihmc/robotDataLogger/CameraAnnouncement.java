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
        
        
    }
    @Override
    public void set(CameraAnnouncement other)
    {
        	name_.setLength(0);
        	name_.append(other.name_);
        	id_ = other.id_;

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

        
    public void setId(int id)
    {
        id_ = id;
    }

    public int getId()
    {
        return id_;
    }

        


	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(CameraAnnouncement data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(CameraAnnouncement data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    if(name_.length() <= 255)
	    cdr.write_type_d(name_);else
	        throw new RuntimeException("name field exceeds the maximum length");

	    cdr.write_type_2(id_);
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	cdr.read_type_d(name_);	

	    	id_ = cdr.read_type_2();	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_d("name", name_);
			    
			    ser.write_type_2("id", id_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			ser.read_type_d("name", name_);	
	    	    
	    			id_ = ser.read_type_2("id");	
	    	    
	}

    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof CameraAnnouncement)) return false;
        CameraAnnouncement otherMyClass = (CameraAnnouncement)other;
        boolean returnedValue = true;

        returnedValue &= us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_);
                
        returnedValue &= this.id_ == otherMyClass.id_;

                

        return returnedValue;
    }
    
     @Override
    public String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("CameraAnnouncement {");
        builder.append("name=");
        builder.append(this.name_);

                builder.append(", ");
        builder.append("id=");
        builder.append(this.id_);

                
        builder.append("}");
		return builder.toString();
    }

    private StringBuilder name_; 
    private int id_; 

}