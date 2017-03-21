package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "YoRegistryDefinition" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class YoRegistryDefinition implements IDLStruct<YoRegistryDefinition>
{
    public YoRegistryDefinition()
    {
        	name_ = new StringBuilder(255); 
        
        
    }
    @Override
    public void set(YoRegistryDefinition other)
    {
        	parent_ = other.parent_;
        	name_.setLength(0);
        	name_.append(other.name_);

    }

    public void setParent(int parent)
    {
        parent_ = parent;
    }

    public int getParent()
    {
        return parent_;
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

        


	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(YoRegistryDefinition data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(YoRegistryDefinition data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    cdr.write_type_2(parent_);

	    if(name_.length() <= 255)
	    cdr.write_type_d(name_);else
	        throw new RuntimeException("name field exceeds the maximum length");
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	parent_ = cdr.read_type_2();	

	    	cdr.read_type_d(name_);	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_2("parent", parent_);
			    
			    ser.write_type_d("name", name_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			parent_ = ser.read_type_2("parent");	
	    	    
	    			ser.read_type_d("name", name_);	
	    	    
	}

    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof YoRegistryDefinition)) return false;
        YoRegistryDefinition otherMyClass = (YoRegistryDefinition)other;
        boolean returnedValue = true;

        returnedValue &= this.parent_ == otherMyClass.parent_;

                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_);
                

        return returnedValue;
    }
    
     @Override
    public String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("YoRegistryDefinition {");
        builder.append("parent=");
        builder.append(this.parent_);

                builder.append(", ");
        builder.append("name=");
        builder.append(this.name_);

                
        builder.append("}");
		return builder.toString();
    }

    private int parent_; 
    private StringBuilder name_; 

}