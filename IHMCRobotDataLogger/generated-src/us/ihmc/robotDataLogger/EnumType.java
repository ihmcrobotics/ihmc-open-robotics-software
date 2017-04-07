package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "EnumType" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class EnumType implements IDLStruct<EnumType>
{
    public EnumType()
    {
        	name_ = new StringBuilder(255); 
        	enumValues_ = new IDLSequence.StringBuilderHolder (128, "type_d");           
        
    }
    @Override
    public void set(EnumType other)
    {
        	name_.setLength(0);
        	name_.append(other.name_);
        	enumValues_.set(other.enumValues_);
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

        

    public IDLSequence.StringBuilderHolder  getEnumValues()
    {
        return enumValues_;
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
	    for(int a = 0; a < 128; ++a)
	    {
	        current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;
	    }
	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(EnumType data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(EnumType data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getEnumValues().size(); ++a)
	    {
	        current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getEnumValues().get(a).length() + 1;
	    }
	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    if(name_.length() <= 255)
	    cdr.write_type_d(name_);else
	        throw new RuntimeException("name field exceeds the maximum length");

	    if(enumValues_.size() <= 128)
	    cdr.write_type_e(enumValues_);else
	        throw new RuntimeException("enumValues field exceeds the maximum length");
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	cdr.read_type_d(name_);	

	    	cdr.read_type_e(enumValues_);	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_d("name", name_);
			    
			    ser.write_type_e("enumValues", enumValues_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			ser.read_type_d("name", name_);	
	    	    
	    			ser.read_type_e("enumValues", enumValues_);	
	    	    
	}

    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof EnumType)) return false;
        EnumType otherMyClass = (EnumType)other;
        boolean returnedValue = true;

        returnedValue &= us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_);
                
        returnedValue &= this.enumValues_.equals(otherMyClass.enumValues_);
                

        return returnedValue;
    }
    
     @Override
    public String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("EnumType {");
        builder.append("name=");
        builder.append(this.name_);

                builder.append(", ");
        builder.append("enumValues=");
        builder.append(this.enumValues_);

                
        builder.append("}");
		return builder.toString();
    }

    private StringBuilder name_; 
    private IDLSequence.StringBuilderHolder  enumValues_; 

}