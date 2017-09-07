package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import java.util.Arrays;

/**
* 
* Definition of the class "EnumType" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class EnumType
{
    public EnumType()
    {
        	name_ = new StringBuilder(255); 
        	enumValues_ = new IDLSequence.StringBuilderHolder (255, "type_d");           
        
    }

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