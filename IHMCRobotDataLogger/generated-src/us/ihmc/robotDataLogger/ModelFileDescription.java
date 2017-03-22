package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "ModelFileDescription" defined in Announcement.idl. 
*
* This file was automatically generated from Announcement.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Announcement.idl instead.
*
*/
public class ModelFileDescription implements IDLStruct<ModelFileDescription>
{
    public ModelFileDescription()
    {
        	name_ = new StringBuilder(255); 
        	resourceDirectories_ = new IDLSequence.StringBuilderHolder (255, "type_d");           
        
    }
    @Override
    public void set(ModelFileDescription other)
    {
        	hasModel_ = other.hasModel_;
        	name_.setLength(0);
        	name_.append(other.name_);
        	resourceDirectories_.set(other.resourceDirectories_);modelFileSize_ = other.modelFileSize_;
        	hasResourceZip_ = other.hasResourceZip_;
        	resourceZipSize_ = other.resourceZipSize_;

    }

    public void setHasModel(boolean hasModel)
    {
        hasModel_ = hasModel;
    }

    public boolean getHasModel()
    {
        return hasModel_;
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

        

    public IDLSequence.StringBuilderHolder  getResourceDirectories()
    {
        return resourceDirectories_;
    }

        
    public void setModelFileSize(int modelFileSize)
    {
        modelFileSize_ = modelFileSize;
    }

    public int getModelFileSize()
    {
        return modelFileSize_;
    }

        
    public void setHasResourceZip(boolean hasResourceZip)
    {
        hasResourceZip_ = hasResourceZip;
    }

    public boolean getHasResourceZip()
    {
        return hasResourceZip_;
    }

        
    public void setResourceZipSize(int resourceZipSize)
    {
        resourceZipSize_ = resourceZipSize;
    }

    public int getResourceZipSize()
    {
        return resourceZipSize_;
    }

        


	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 255; ++a)
	    {
	        current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;
	    }
	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(ModelFileDescription data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(ModelFileDescription data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getResourceDirectories().size(); ++a)
	    {
	        current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getResourceDirectories().get(a).length() + 1;
	    }
	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);

	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    cdr.write_type_7(hasModel_);

	    if(name_.length() <= 255)
	    cdr.write_type_d(name_);else
	        throw new RuntimeException("name field exceeds the maximum length");

	    if(resourceDirectories_.size() <= 255)
	    cdr.write_type_e(resourceDirectories_);else
	        throw new RuntimeException("resourceDirectories field exceeds the maximum length");

	    cdr.write_type_2(modelFileSize_);

	    cdr.write_type_7(hasResourceZip_);

	    cdr.write_type_2(resourceZipSize_);
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	hasModel_ = cdr.read_type_7();	

	    	cdr.read_type_d(name_);	

	    	cdr.read_type_e(resourceDirectories_);	

	    	modelFileSize_ = cdr.read_type_2();	

	    	hasResourceZip_ = cdr.read_type_7();	

	    	resourceZipSize_ = cdr.read_type_2();	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_7("hasModel", hasModel_);
			    
			    ser.write_type_d("name", name_);
			    
			    ser.write_type_e("resourceDirectories", resourceDirectories_);
			    
			    ser.write_type_2("modelFileSize", modelFileSize_);
			    
			    ser.write_type_7("hasResourceZip", hasResourceZip_);
			    
			    ser.write_type_2("resourceZipSize", resourceZipSize_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			hasModel_ = ser.read_type_7("hasModel");	
	    	    
	    			ser.read_type_d("name", name_);	
	    	    
	    			ser.read_type_e("resourceDirectories", resourceDirectories_);	
	    	    
	    			modelFileSize_ = ser.read_type_2("modelFileSize");	
	    	    
	    			hasResourceZip_ = ser.read_type_7("hasResourceZip");	
	    	    
	    			resourceZipSize_ = ser.read_type_2("resourceZipSize");	
	    	    
	}

    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof ModelFileDescription)) return false;
        ModelFileDescription otherMyClass = (ModelFileDescription)other;
        boolean returnedValue = true;

        returnedValue &= this.hasModel_ == otherMyClass.hasModel_;

                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_);
                
        returnedValue &= this.resourceDirectories_.equals(otherMyClass.resourceDirectories_);
                
        returnedValue &= this.modelFileSize_ == otherMyClass.modelFileSize_;

                
        returnedValue &= this.hasResourceZip_ == otherMyClass.hasResourceZip_;

                
        returnedValue &= this.resourceZipSize_ == otherMyClass.resourceZipSize_;

                

        return returnedValue;
    }
    
     @Override
    public String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("ModelFileDescription {");
        builder.append("hasModel=");
        builder.append(this.hasModel_);

                builder.append(", ");
        builder.append("name=");
        builder.append(this.name_);

                builder.append(", ");
        builder.append("resourceDirectories=");
        builder.append(this.resourceDirectories_);

                builder.append(", ");
        builder.append("modelFileSize=");
        builder.append(this.modelFileSize_);

                builder.append(", ");
        builder.append("hasResourceZip=");
        builder.append(this.hasResourceZip_);

                builder.append(", ");
        builder.append("resourceZipSize=");
        builder.append(this.resourceZipSize_);

                
        builder.append("}");
		return builder.toString();
    }

    private boolean hasModel_; 
    private StringBuilder name_; 
    private IDLSequence.StringBuilderHolder  resourceDirectories_; 
    private int modelFileSize_; 
    private boolean hasResourceZip_; 
    private int resourceZipSize_; 

}