package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "DynamicGraphicMessage" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class DynamicGraphicMessage implements IDLStruct<DynamicGraphicMessage>
{
    public DynamicGraphicMessage()
    {
        	name_ = new StringBuilder(255); 
        	yo_index_ = new IDLSequence.Integer (255, "type_2");
        	constants_ = new IDLSequence.Double (255, "type_6");
        	appearance_ = new us.ihmc.robotDataLogger.AppearanceDefinitionMessage();list_name_ = new StringBuilder(255); 
        
        
    }
    @Override
    public void set(DynamicGraphicMessage other)
    {
        	type_ = other.type_;
        	name_.setLength(0);
        	name_.append(other.name_);
        	yo_index_.set(other.yo_index_);constants_.set(other.constants_);appearance_.set(other.appearance_);list_name_.setLength(0);
        	list_name_.append(other.list_name_);

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

        

    public IDLSequence.Integer  getYo_index()
    {
        return yo_index_;
    }

        

    public IDLSequence.Double  getConstants()
    {
        return constants_;
    }

        

    public us.ihmc.robotDataLogger.AppearanceDefinitionMessage getAppearance()
    {
        return appearance_;
    }

        
        public void setList_name(String list_name)
        {
        	list_name_.setLength(0);
        	list_name_.append(list_name);
        }
        
        public String getList_nameAsString()
        {
        	return getList_name().toString();
        }

    public StringBuilder getList_name()
    {
        return list_name_;
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

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    current_alignment += (255 * 4) + CDR.alignment(current_alignment, 4);


	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    current_alignment += (255 * 8) + CDR.alignment(current_alignment, 8);


	    current_alignment += us.ihmc.robotDataLogger.AppearanceDefinitionMessage.getMaxCdrSerializedSize(current_alignment);
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(DynamicGraphicMessage data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(DynamicGraphicMessage data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    current_alignment += (data.getYo_index().size() * 4) + CDR.alignment(current_alignment, 4);


	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    current_alignment += (data.getConstants().size() * 8) + CDR.alignment(current_alignment, 8);


	    current_alignment += us.ihmc.robotDataLogger.AppearanceDefinitionMessage.getCdrSerializedSize(data.getAppearance(), current_alignment);
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getList_name().length() + 1;

	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    cdr.write_type_2(type_);

	    if(name_.length() <= 255)
	    cdr.write_type_d(name_);else
	        throw new RuntimeException("name field exceeds the maximum length");

	    if(yo_index_.size() <= 255)
	    cdr.write_type_e(yo_index_);else
	        throw new RuntimeException("yo_index field exceeds the maximum length");

	    if(constants_.size() <= 255)
	    cdr.write_type_e(constants_);else
	        throw new RuntimeException("constants field exceeds the maximum length");

	    cdr.write_type_a(appearance_);

	    if(list_name_.length() <= 255)
	    cdr.write_type_d(list_name_);else
	        throw new RuntimeException("list_name field exceeds the maximum length");
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	type_ = cdr.read_type_2();	

	    	cdr.read_type_d(name_);	

	    	cdr.read_type_e(yo_index_);	

	    	cdr.read_type_e(constants_);	

	    	cdr.read_type_a(appearance_);	

	    	cdr.read_type_d(list_name_);	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_2("type", type_);
			    
			    ser.write_type_d("name", name_);
			    
			    ser.write_type_e("yo_index", yo_index_);
			    
			    ser.write_type_e("constants", constants_);
			    
			    ser.write_type_a("appearance", appearance_);
			    
			    ser.write_type_d("list_name", list_name_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			type_ = ser.read_type_2("type");	
	    	    
	    			ser.read_type_d("name", name_);	
	    	    
	    			ser.read_type_e("yo_index", yo_index_);	
	    	    
	    			ser.read_type_e("constants", constants_);	
	    	    
	    			ser.read_type_a("appearance", appearance_);	
	    	    
	    			ser.read_type_d("list_name", list_name_);	
	    	    
	}

    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof DynamicGraphicMessage)) return false;
        DynamicGraphicMessage otherMyClass = (DynamicGraphicMessage)other;
        boolean returnedValue = true;

        returnedValue &= this.type_ == otherMyClass.type_;

                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_);
                
        returnedValue &= this.yo_index_.equals(otherMyClass.yo_index_);
                
        returnedValue &= this.constants_.equals(otherMyClass.constants_);
                
        returnedValue &= this.appearance_.equals(otherMyClass.appearance_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.list_name_, otherMyClass.list_name_);
                

        return returnedValue;
    }
    
     @Override
    public String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("DynamicGraphicMessage {");
        builder.append("type=");
        builder.append(this.type_);

                builder.append(", ");
        builder.append("name=");
        builder.append(this.name_);

                builder.append(", ");
        builder.append("yo_index=");
        builder.append(this.yo_index_);

                builder.append(", ");
        builder.append("constants=");
        builder.append(this.constants_);

                builder.append(", ");
        builder.append("appearance=");
        builder.append(this.appearance_);

                builder.append(", ");
        builder.append("list_name=");
        builder.append(this.list_name_);

                
        builder.append("}");
		return builder.toString();
    }

    private int type_; 
    private StringBuilder name_; 
    private IDLSequence.Integer  yo_index_; 
    private IDLSequence.Double  constants_; 
    private us.ihmc.robotDataLogger.AppearanceDefinitionMessage appearance_; 
    private StringBuilder list_name_; 

}