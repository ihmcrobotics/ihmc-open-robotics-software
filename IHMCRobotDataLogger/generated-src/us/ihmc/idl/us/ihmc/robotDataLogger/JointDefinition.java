package us.ihmc.idl.us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "JointDefinition" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class JointDefinition implements IDLStruct<JointDefinition>
{
    public JointDefinition()
    {
        	name_ = new StringBuilder(255); 
        
        
    }
    @Override
    public void set(JointDefinition other)
    {
        	name_.setLength(0);
        	name_.append(other.name_);
        	type_ = other.type_;

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

        
    public void setType(us.ihmc.idl.us.ihmc.robotDataLogger.JointType type)
    {
        type_ = type;
    }

    public us.ihmc.idl.us.ihmc.robotDataLogger.JointType getType()
    {
        return type_;
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


	public final static int getCdrSerializedSize(JointDefinition data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(JointDefinition data, int current_alignment)
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

	    cdr.write_type_c(type_.ordinal());

	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	cdr.read_type_d(name_);	

	    	type_ = us.ihmc.idl.us.ihmc.robotDataLogger.JointType.values[cdr.read_type_c()];
	    	
	}

    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof JointDefinition)) return false;
        JointDefinition otherMyClass = (JointDefinition)other;
        boolean returnedValue = true;

        returnedValue &= us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_);
                
        returnedValue &= this.type_ == otherMyClass.type_;

                

        return returnedValue;
    }
    
     @Override
    public String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("JointDefinition {");
        builder.append("name=");
        builder.append(this.name_);

                builder.append(", ");
        builder.append("type=");
        builder.append(this.type_);

                
        builder.append("}");
		return builder.toString();
    }

    private StringBuilder name_; 
    private us.ihmc.idl.us.ihmc.robotDataLogger.JointType type_; 

}