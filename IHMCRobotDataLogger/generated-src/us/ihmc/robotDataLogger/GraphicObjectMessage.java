package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "GraphicObjectMessage" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class GraphicObjectMessage implements IDLStruct<GraphicObjectMessage>
{
    public GraphicObjectMessage()
    {
        	name_ = new StringBuilder(255); 
        	yoVariableIndex_ = new IDLSequence.Integer (128, "type_3");
        	constants_ = new IDLSequence.Double (128, "type_6");
        	appearance_ = new us.ihmc.robotDataLogger.AppearanceDefinitionMessage();listName_ = new StringBuilder(255); 
        
        
    }
    @Override
    public void set(GraphicObjectMessage other)
    {
        	type_ = other.type_;
        	name_.setLength(0);
        	name_.append(other.name_);
        	yoVariableIndex_.set(other.yoVariableIndex_);constants_.set(other.constants_);appearance_.set(other.appearance_);listName_.setLength(0);
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
        
        public String getNameAsString()
        {
        	return getName().toString();
        }

    public StringBuilder getName()
    {
        return name_;
    }

        

    public IDLSequence.Integer  getYoVariableIndex()
    {
        return yoVariableIndex_;
    }

        

    public IDLSequence.Double  getConstants()
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
        
        public String getListNameAsString()
        {
        	return getListName().toString();
        }

    public StringBuilder getListName()
    {
        return listName_;
    }

        


	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 2 + CDR.alignment(current_alignment, 2);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    current_alignment += (128 * 2) + CDR.alignment(current_alignment, 2);


	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    current_alignment += (128 * 8) + CDR.alignment(current_alignment, 8);


	    current_alignment += us.ihmc.robotDataLogger.AppearanceDefinitionMessage.getMaxCdrSerializedSize(current_alignment);
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(GraphicObjectMessage data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(GraphicObjectMessage data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 2 + CDR.alignment(current_alignment, 2);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    current_alignment += (data.getYoVariableIndex().size() * 2) + CDR.alignment(current_alignment, 2);


	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    current_alignment += (data.getConstants().size() * 8) + CDR.alignment(current_alignment, 8);


	    current_alignment += us.ihmc.robotDataLogger.AppearanceDefinitionMessage.getCdrSerializedSize(data.getAppearance(), current_alignment);
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getListName().length() + 1;

	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    cdr.write_type_3(type_);

	    if(name_.length() <= 255)
	    cdr.write_type_d(name_);else
	        throw new RuntimeException("name field exceeds the maximum length");

	    if(yoVariableIndex_.size() <= 128)
	    cdr.write_type_e(yoVariableIndex_);else
	        throw new RuntimeException("yoVariableIndex field exceeds the maximum length");

	    if(constants_.size() <= 128)
	    cdr.write_type_e(constants_);else
	        throw new RuntimeException("constants field exceeds the maximum length");

	    cdr.write_type_a(appearance_);

	    if(listName_.length() <= 255)
	    cdr.write_type_d(listName_);else
	        throw new RuntimeException("listName field exceeds the maximum length");
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	type_ = cdr.read_type_3();	

	    	cdr.read_type_d(name_);	

	    	cdr.read_type_e(yoVariableIndex_);	

	    	cdr.read_type_e(constants_);	

	    	cdr.read_type_a(appearance_);	

	    	cdr.read_type_d(listName_);	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_3("type", type_);
			    
			    ser.write_type_d("name", name_);
			    
			    ser.write_type_e("yoVariableIndex", yoVariableIndex_);
			    
			    ser.write_type_e("constants", constants_);
			    
			    ser.write_type_a("appearance", appearance_);
			    
			    ser.write_type_d("listName", listName_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			type_ = ser.read_type_3("type");	
	    	    
	    			ser.read_type_d("name", name_);	
	    	    
	    			ser.read_type_e("yoVariableIndex", yoVariableIndex_);	
	    	    
	    			ser.read_type_e("constants", constants_);	
	    	    
	    			ser.read_type_a("appearance", appearance_);	
	    	    
	    			ser.read_type_d("listName", listName_);	
	    	    
	}

    @Override
    public boolean equals(Object other)
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
    public String toString()
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
    private StringBuilder name_; 
    private IDLSequence.Integer  yoVariableIndex_; 
    private IDLSequence.Double  constants_; 
    private us.ihmc.robotDataLogger.AppearanceDefinitionMessage appearance_; 
    private StringBuilder listName_; 

}