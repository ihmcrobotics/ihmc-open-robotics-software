package us.ihmc.robotDataLogger;
/**
* 
* Definition of the class "YoVariableDefinition" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class YoVariableDefinition
{
    public YoVariableDefinition()
    {
        	name_ = new java.lang.StringBuilder(255); 
        	description_ = new java.lang.StringBuilder(255); 
        
        
    }

    public void set(YoVariableDefinition other)
    {
        	name_.setLength(0);
        	name_.append(other.name_);
        	description_.setLength(0);
        	description_.append(other.description_);
        	type_ = other.type_;
        	registry_ = other.registry_;
        	enumType_ = other.enumType_;
        	allowNullValues_ = other.allowNullValues_;
        	isParameter_ = other.isParameter_;
        	min_ = other.min_;
        	max_ = other.max_;

    }

        public void setName(String name)
        {
        	name_.setLength(0);
        	name_.append(name);
        }
        
        public java.lang.String getNameAsString()
        {
        	return getName().toString();
        }

    public java.lang.StringBuilder getName()
    {
        return name_;
    }

        
        public void setDescription(String description)
        {
        	description_.setLength(0);
        	description_.append(description);
        }
        
        public java.lang.String getDescriptionAsString()
        {
        	return getDescription().toString();
        }

    public java.lang.StringBuilder getDescription()
    {
        return description_;
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

        
    public void setIsParameter(boolean isParameter)
    {
        isParameter_ = isParameter;
    }

    public boolean getIsParameter()
    {
        return isParameter_;
    }

        
    public void setMin(double min)
    {
        min_ = min;
    }

    public double getMin()
    {
        return min_;
    }

        
    public void setMax(double max)
    {
        max_ = max;
    }

    public double getMax()
    {
        return max_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof YoVariableDefinition)) return false;
        YoVariableDefinition otherMyClass = (YoVariableDefinition)other;
        boolean returnedValue = true;

        returnedValue &= us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.description_, otherMyClass.description_);
                
        returnedValue &= this.type_ == otherMyClass.type_;

                
        returnedValue &= this.registry_ == otherMyClass.registry_;

                
        returnedValue &= this.enumType_ == otherMyClass.enumType_;

                
        returnedValue &= this.allowNullValues_ == otherMyClass.allowNullValues_;

                
        returnedValue &= this.isParameter_ == otherMyClass.isParameter_;

                
        returnedValue &= this.min_ == otherMyClass.min_;

                
        returnedValue &= this.max_ == otherMyClass.max_;

                

        return returnedValue;
    }
    
     @Override
    public java.lang.String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("YoVariableDefinition {");
        builder.append("name=");
        builder.append(this.name_);

                builder.append(", ");
        builder.append("description=");
        builder.append(this.description_);

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

                builder.append(", ");
        builder.append("isParameter=");
        builder.append(this.isParameter_);

                builder.append(", ");
        builder.append("min=");
        builder.append(this.min_);

                builder.append(", ");
        builder.append("max=");
        builder.append(this.max_);

                
        builder.append("}");
		return builder.toString();
    }

    private java.lang.StringBuilder name_; 
    private java.lang.StringBuilder description_; 
    private us.ihmc.robotDataLogger.YoType type_; 
    private int registry_; 
    private int enumType_; 
    private boolean allowNullValues_; 
    private boolean isParameter_; 
    private double min_; 
    private double max_; 

}