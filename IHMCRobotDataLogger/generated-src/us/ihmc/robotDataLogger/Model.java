package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "Model" defined in LogProperties.idl. 
*
* This file was automatically generated from LogProperties.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogProperties.idl instead.
*
*/
public class Model implements IDLStruct<Model>
{
    public Model()
    {
        	loader_ = new StringBuilder(255); 
        	path_ = new StringBuilder(255); 
        	name_ = new StringBuilder(255); 
        	resourceBundle_ = new StringBuilder(255); 
        	resourceDirectoriesList_ = new IDLSequence.StringBuilderHolder (255, "type_d");           
        
    }
    @Override
    public void set(Model other)
    {
        	loader_.setLength(0);
        	loader_.append(other.loader_);
        	path_.setLength(0);
        	path_.append(other.path_);
        	name_.setLength(0);
        	name_.append(other.name_);
        	resourceBundle_.setLength(0);
        	resourceBundle_.append(other.resourceBundle_);
        	resourceDirectoriesList_.set(other.resourceDirectoriesList_);
    }

        public void setLoader(String loader)
        {
        	loader_.setLength(0);
        	loader_.append(loader);
        }
        
        public String getLoaderAsString()
        {
        	return getLoader().toString();
        }

    public StringBuilder getLoader()
    {
        return loader_;
    }

        
        public void setPath(String path)
        {
        	path_.setLength(0);
        	path_.append(path);
        }
        
        public String getPathAsString()
        {
        	return getPath().toString();
        }

    public StringBuilder getPath()
    {
        return path_;
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

        
        public void setResourceBundle(String resourceBundle)
        {
        	resourceBundle_.setLength(0);
        	resourceBundle_.append(resourceBundle);
        }
        
        public String getResourceBundleAsString()
        {
        	return getResourceBundle().toString();
        }

    public StringBuilder getResourceBundle()
    {
        return resourceBundle_;
    }

        

    public IDLSequence.StringBuilderHolder  getResourceDirectoriesList()
    {
        return resourceDirectoriesList_;
    }

        


	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 255; ++a)
	    {
	        current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;
	    }
	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(Model data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(Model data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getLoader().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getPath().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getResourceBundle().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getResourceDirectoriesList().size(); ++a)
	    {
	        current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getResourceDirectoriesList().get(a).length() + 1;
	    }
	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    if(loader_.length() <= 255)
	    cdr.write_type_d(loader_);else
	        throw new RuntimeException("loader field exceeds the maximum length");

	    if(path_.length() <= 255)
	    cdr.write_type_d(path_);else
	        throw new RuntimeException("path field exceeds the maximum length");

	    if(name_.length() <= 255)
	    cdr.write_type_d(name_);else
	        throw new RuntimeException("name field exceeds the maximum length");

	    if(resourceBundle_.length() <= 255)
	    cdr.write_type_d(resourceBundle_);else
	        throw new RuntimeException("resourceBundle field exceeds the maximum length");

	    if(resourceDirectoriesList_.size() <= 255)
	    cdr.write_type_e(resourceDirectoriesList_);else
	        throw new RuntimeException("resourceDirectoriesList field exceeds the maximum length");
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	cdr.read_type_d(loader_);	

	    	cdr.read_type_d(path_);	

	    	cdr.read_type_d(name_);	

	    	cdr.read_type_d(resourceBundle_);	

	    	cdr.read_type_e(resourceDirectoriesList_);	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_d("loader", loader_);
			    
			    ser.write_type_d("path", path_);
			    
			    ser.write_type_d("name", name_);
			    
			    ser.write_type_d("resourceBundle", resourceBundle_);
			    
			    ser.write_type_e("resourceDirectoriesList", resourceDirectoriesList_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			ser.read_type_d("loader", loader_);	
	    	    
	    			ser.read_type_d("path", path_);	
	    	    
	    			ser.read_type_d("name", name_);	
	    	    
	    			ser.read_type_d("resourceBundle", resourceBundle_);	
	    	    
	    			ser.read_type_e("resourceDirectoriesList", resourceDirectoriesList_);	
	    	    
	}

    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof Model)) return false;
        Model otherMyClass = (Model)other;
        boolean returnedValue = true;

        returnedValue &= us.ihmc.idl.IDLTools.equals(this.loader_, otherMyClass.loader_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.path_, otherMyClass.path_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.resourceBundle_, otherMyClass.resourceBundle_);
                
        returnedValue &= this.resourceDirectoriesList_.equals(otherMyClass.resourceDirectoriesList_);
                

        return returnedValue;
    }
    
     @Override
    public String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("Model {");
        builder.append("loader=");
        builder.append(this.loader_);

                builder.append(", ");
        builder.append("path=");
        builder.append(this.path_);

                builder.append(", ");
        builder.append("name=");
        builder.append(this.name_);

                builder.append(", ");
        builder.append("resourceBundle=");
        builder.append(this.resourceBundle_);

                builder.append(", ");
        builder.append("resourceDirectoriesList=");
        builder.append(this.resourceDirectoriesList_);

                
        builder.append("}");
		return builder.toString();
    }

    private StringBuilder loader_; 
    private StringBuilder path_; 
    private StringBuilder name_; 
    private StringBuilder resourceBundle_; 
    private IDLSequence.StringBuilderHolder  resourceDirectoriesList_; 

}