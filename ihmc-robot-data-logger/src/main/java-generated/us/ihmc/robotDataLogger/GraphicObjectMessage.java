package us.ihmc.robotDataLogger;
/**
* 
* Definition of the class "GraphicObjectMessage" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class GraphicObjectMessage
{
    public GraphicObjectMessage()
    {
        	name_ = new java.lang.StringBuilder(255); 
        	yoVariableIndex_ = new us.ihmc.idl.IDLSequence.Integer (1024, "type_3");
        	constants_ = new us.ihmc.idl.IDLSequence.Double (128, "type_6");
        	appearance_ = new us.ihmc.robotDataLogger.AppearanceDefinitionMessage();listName_ = new java.lang.StringBuilder(255); 
        
        
    }

    public void set(GraphicObjectMessage other)
    {
        	type_ = other.type_;
        	name_.setLength(0);
        	name_.append(other.name_);
            yoVariableIndex_.set(other.yoVariableIndex_);	constants_.set(other.constants_);	us.ihmc.robotDataLogger.AppearanceDefinitionMessagePubSubType.staticCopy(other.appearance_, appearance_);listName_.setLength(0);
        	listName_.append(other.listName_);

    }

    public void setType(int type)
    {
        type_ = type;
    }

    public int getType()
    {
        return type_;
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

        

    public us.ihmc.idl.IDLSequence.Integer  getYoVariableIndex()
    {
        return yoVariableIndex_;
    }

        

    public us.ihmc.idl.IDLSequence.Double  getConstants()
    {
        return constants_;
    }

        

    public us.ihmc.robotDataLogger.AppearanceDefinitionMessage getAppearance()
    {
        return appearance_;
    }

        
        public void setListName(String listName)
        {
        	listName_.setLength(0);
        	listName_.append(listName);
        }
        
        public java.lang.String getListNameAsString()
        {
        	return getListName().toString();
        }

    public java.lang.StringBuilder getListName()
    {
        return listName_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof GraphicObjectMessage)) return false;
        GraphicObjectMessage otherMyClass = (GraphicObjectMessage)other;
        boolean returnedValue = true;

        returnedValue &= this.type_ == otherMyClass.type_;

                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_);
                
        returnedValue &= this.yoVariableIndex_.equals(otherMyClass.yoVariableIndex_);
                
        returnedValue &= this.constants_.equals(otherMyClass.constants_);
                
        returnedValue &= this.appearance_.equals(otherMyClass.appearance_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.listName_, otherMyClass.listName_);
                

        return returnedValue;
    }
    
     @Override
    public java.lang.String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("GraphicObjectMessage {");
        builder.append("type=");
        builder.append(this.type_);

                builder.append(", ");
        builder.append("name=");
        builder.append(this.name_);

                builder.append(", ");
        builder.append("yoVariableIndex=");
        builder.append(this.yoVariableIndex_);

                builder.append(", ");
        builder.append("constants=");
        builder.append(this.constants_);

                builder.append(", ");
        builder.append("appearance=");
        builder.append(this.appearance_);

                builder.append(", ");
        builder.append("listName=");
        builder.append(this.listName_);

                
        builder.append("}");
		return builder.toString();
    }

    private int type_; 
    private java.lang.StringBuilder name_; 
    private us.ihmc.idl.IDLSequence.Integer  yoVariableIndex_; 
    private us.ihmc.idl.IDLSequence.Double  constants_; 
    private us.ihmc.robotDataLogger.AppearanceDefinitionMessage appearance_; 
    private java.lang.StringBuilder listName_; 

}