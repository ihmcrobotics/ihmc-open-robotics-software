package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
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
        
        
    }
    @Override
    public void set(YoVariableDefinition other)
    {
        	name_.setLength(0);
        	name_.append(other.name_);
        	type_ = other.type_;
        	registry_ = other.registry_;
        	enumType_ = other.enumType_;
        	allowNullValues_ = other.allowNullValues_;

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

        
    public void setType(us.ihmc.robotDataLogger.YoType type)
    {
        type_ = type;
    }

    public us.ihmc.robotDataLogger.YoType getType()
    {
        return type_;
    }

        
    public void setRegistry(int registry)
    {
        registry_ = registry;
    }

    public int getRegistry()
    {
        return registry_;
    }

        
    public void setEnumType(int enumType)
    {
        enumType_ = enumType;
    }

    public int getEnumType()
    {
        return enumType_;
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

	    current_alignment += 2 + CDR.alignment(current_alignment, 2);

	    current_alignment += 2 + CDR.alignment(current_alignment, 2);

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

	    current_alignment += 2 + CDR.alignment(current_alignment, 2);

	    current_alignment += 2 + CDR.alignment(current_alignment, 2);

	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    if(name_.length() <= 255)
	    cdr.write_type_d(name_);else
	        throw new RuntimeException("name field exceeds the maximum length");

	    cdr.write_type_c(type_.ordinal());


	    cdr.write_type_3(registry_);

	    cdr.write_type_3(enumType_);

	    cdr.write_type_7(allowNullValues_);
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	cdr.read_type_d(name_);	

	    	type_ = us.ihmc.robotDataLogger.YoType.values[cdr.read_type_c()];
	    	

	    	registry_ = cdr.read_type_3();	

	    	enumType_ = cdr.read_type_3();	

	    	allowNullValues_ = cdr.read_type_7();	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_d("name", name_);
			    
			    ser.write_type_c("type", type_);
			    
			    ser.write_type_3("registry", registry_);
			    
			    ser.write_type_3("enumType", enumType_);
			    
			    ser.write_type_7("allowNullValues", allowNullValues_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			ser.read_type_d("name", name_);	
	    	    
	    			type_ = (us.ihmc.robotDataLogger.YoType)ser.read_type_c("type", us.ihmc.robotDataLogger.YoType.class);
	    	
	    	    
	    			registry_ = ser.read_type_3("registry");	
	    	    
	    			enumType_ = ser.read_type_3("enumType");	
	    	    
	    			allowNullValues_ = ser.read_type_7("allowNullValues");	
	    	    
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
                
        returnedValue &= this.type_ == otherMyClass.type_;

                
        returnedValue &= this.registry_ == otherMyClass.registry_;

                
        returnedValue &= this.enumType_ == otherMyClass.enumType_;

                
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
        builder.append("type=");
        builder.append(this.type_);

                builder.append(", ");
        builder.append("registry=");
        builder.append(this.registry_);

                builder.append(", ");
        builder.append("enumType=");
        builder.append(this.enumType_);

                builder.append(", ");
        builder.append("allowNullValues=");
        builder.append(this.allowNullValues_);

                
        builder.append("}");
		return builder.toString();
    }

    private StringBuilder name_; 
    private us.ihmc.robotDataLogger.YoType type_; 
    private int registry_; 
    private int enumType_; 
    private boolean allowNullValues_; 

}