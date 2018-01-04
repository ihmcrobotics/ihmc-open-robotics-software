package us.ihmc.robotDataLogger;
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
        	loader_ = new java.lang.StringBuilder(255); 
        	path_ = new java.lang.StringBuilder(255); 
        	name_ = new java.lang.StringBuilder(255); 
        	resourceBundle_ = new java.lang.StringBuilder(255); 
        	resourceDirectoriesList_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (255, "type_d");           
        
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
        
        public java.lang.String getLoaderAsString()
        {
        	return getLoader().toString();
        }

    public java.lang.StringBuilder getLoader()
    {
        return loader_;
    }

        
        public void setPath(String path)
        {
        	path_.setLength(0);
        	path_.append(path);
        }
        
        public java.lang.String getPathAsString()
        {
        	return getPath().toString();
        }

    public java.lang.StringBuilder getPath()
    {
        return path_;
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

        
        public void setResourceBundle(String resourceBundle)
        {
        	resourceBundle_.setLength(0);
        	resourceBundle_.append(resourceBundle);
        }
        
        public java.lang.String getResourceBundleAsString()
        {
        	return getResourceBundle().toString();
        }

    public java.lang.StringBuilder getResourceBundle()
    {
        return resourceBundle_;
    }

        

    public us.ihmc.idl.IDLSequence.StringBuilderHolder  getResourceDirectoriesList()
    {
        return resourceDirectoriesList_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
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
    public java.lang.String toString()
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

    private java.lang.StringBuilder loader_; 
    private java.lang.StringBuilder path_; 
    private java.lang.StringBuilder name_; 
    private java.lang.StringBuilder resourceBundle_; 
    private us.ihmc.idl.IDLSequence.StringBuilderHolder  resourceDirectoriesList_; 

}