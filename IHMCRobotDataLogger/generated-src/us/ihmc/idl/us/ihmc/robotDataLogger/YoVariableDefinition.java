package us.ihmc.idl.us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "YoVariableDefinition" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class YoVariableDefinition implements IDLStruct<YoVariableDefinition>
{
    public YoVariableDefinition()
    {
        	name_ = new StringBuilder(255); 
        	enumValues_ = new IDLSequence.StringBuilderHolder (255, "type_d");           
        
    }
    @Override
    public void set(YoVariableDefinition other)
    {
        	name_.setLength(0);
        	name_.append(other.name_);
        	registry_ = other.registry_;
        	type_ = other.type_;
        	enumValues_.set(other.enumValues_);allowNullValues_ = other.allowNullValues_;

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

        
    public void setRegistry(int registry)
    {
        registry_ = registry;
    }

    public int getRegistry()
    {
        return registry_;
    }

        
    public void setType(us.ihmc.idl.us.ihmc.robotDataLogger.YoType type)
    {
        type_ = type;
    }

    public us.ihmc.idl.us.ihmc.robotDataLogger.YoType getType()
    {
        return type_;
    }

        

    public IDLSequence.StringBuilderHolder  getEnumValues()
    {
        return enumValues_;
    }

        
    public void setAllowNullValues(boolean allowNullValues)
    {
        allowNullValues_ = allowNullValues;
    }

    public boolean getAllowNullValues()
    {
        return allowNullValues_;
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

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 255; ++a)
	    {
	        current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;
	    }
	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(YoVariableDefinition data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(YoVariableDefinition data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getEnumValues().size(); ++a)
	    {
	        current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getEnumValues().get(a).length() + 1;
	    }
	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    if(name_.length() <= 255)
	    cdr.write_type_d(name_);else
	        throw new RuntimeException("name field exceeds the maximum length");

	    cdr.write_type_2(registry_);

	    cdr.write_type_c(type_.ordinal());


	    if(enumValues_.size() <= 255)
	    cdr.write_type_e(enumValues_);else
	        throw new RuntimeException("enumValues field exceeds the maximum length");

	    cdr.write_type_7(allowNullValues_);
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	cdr.read_type_d(name_);	

	    	registry_ = cdr.read_type_2();	

	    	type_ = us.ihmc.idl.us.ihmc.robotDataLogger.YoType.values[cdr.read_type_c()];
	    	

	    	cdr.read_type_e(enumValues_);	

	    	allowNullValues_ = cdr.read_type_7();	
	}

    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof YoVariableDefinition)) return false;
        YoVariableDefinition otherMyClass = (YoVariableDefinition)other;
        boolean returnedValue = true;

        returnedValue &= us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_);
                
        returnedValue &= this.registry_ == otherMyClass.registry_;

                
        returnedValue &= this.type_ == otherMyClass.type_;

                
        returnedValue &= this.enumValues_.equals(otherMyClass.enumValues_);
                
        returnedValue &= this.allowNullValues_ == otherMyClass.allowNullValues_;

                

        return returnedValue;
    }
    
     @Override
    public String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("YoVariableDefinition {");
        builder.append("name=");
        builder.append(this.name_);

                builder.append(", ");
        builder.append("registry=");
        builder.append(this.registry_);

                builder.append(", ");
        builder.append("type=");
        builder.append(this.type_);

                builder.append(", ");
        builder.append("enumValues=");
        builder.append(this.enumValues_);

                builder.append(", ");
        builder.append("allowNullValues=");
        builder.append(this.allowNullValues_);

                
        builder.append("}");
		return builder.toString();
    }

    private StringBuilder name_; 
    private int registry_; 
    private us.ihmc.idl.us.ihmc.robotDataLogger.YoType type_; 
    private IDLSequence.StringBuilderHolder  enumValues_; 
    private boolean allowNullValues_; 

}