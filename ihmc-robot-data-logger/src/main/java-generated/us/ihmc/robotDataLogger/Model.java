package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import java.util.Arrays;

/**
* 
* Definition of the class "Model" defined in LogProperties.idl. 
*
* This file was automatically generated from LogProperties.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogProperties.idl instead.
*
*/
public class Model
{
    public Model()
    {
        	loader_ = new StringBuilder(255); 
        	path_ = new StringBuilder(255); 
        	name_ = new StringBuilder(255); 
        	resourceBundle_ = new StringBuilder(255); 
        	resourceDirectoriesList_ = new IDLSequence.StringBuilderHolder (255, "type_d");           
        
    }

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