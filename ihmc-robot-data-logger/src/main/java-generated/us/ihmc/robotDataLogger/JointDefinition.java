package us.ihmc.robotDataLogger;
/**
* 
* Definition of the class "JointDefinition" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class JointDefinition
{
    public JointDefinition()
    {
        	name_ = new java.lang.StringBuilder(255); 
        
        
    }

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
        
        public java.lang.String getNameAsString()
        {
        	return getName().toString();
        }

    public java.lang.StringBuilder getName()
    {
        return name_;
    }

        
    public void setType(us.ihmc.robotDataLogger.JointType type)
    {
        type_ = type;
    }

    public us.ihmc.robotDataLogger.JointType getType()
    {
        return type_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
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
    public java.lang.String toString()
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

    private java.lang.StringBuilder name_; 
    private us.ihmc.robotDataLogger.JointType type_; 

}