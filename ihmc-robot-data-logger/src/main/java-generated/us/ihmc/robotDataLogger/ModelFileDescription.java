package us.ihmc.robotDataLogger;
/**
* 
* Definition of the class "ModelFileDescription" defined in Announcement.idl. 
*
* This file was automatically generated from Announcement.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Announcement.idl instead.
*
*/
public class ModelFileDescription
{
    public ModelFileDescription()
    {
        	name_ = new java.lang.StringBuilder(255); 
        	modelLoaderClass_ = new java.lang.StringBuilder(255); 
        	resourceDirectories_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (255, "type_d");           
        
    }

    public void set(ModelFileDescription other)
    {
        	hasModel_ = other.hasModel_;
        	name_.setLength(0);
        	name_.append(other.name_);
        	modelLoaderClass_.setLength(0);
        	modelLoaderClass_.append(other.modelLoaderClass_);
            resourceDirectories_.set(other.resourceDirectories_);	modelFileSize_ = other.modelFileSize_;
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
        
        public java.lang.String getNameAsString()
        {
        	return getName().toString();
        }

    public java.lang.StringBuilder getName()
    {
        return name_;
    }

        
        public void setModelLoaderClass(String modelLoaderClass)
        {
        	modelLoaderClass_.setLength(0);
        	modelLoaderClass_.append(modelLoaderClass);
        }
        
        public java.lang.String getModelLoaderClassAsString()
        {
        	return getModelLoaderClass().toString();
        }

    public java.lang.StringBuilder getModelLoaderClass()
    {
        return modelLoaderClass_;
    }

        

    public us.ihmc.idl.IDLSequence.StringBuilderHolder  getResourceDirectories()
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

        




    @Override
    public boolean equals(java.lang.Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof ModelFileDescription)) return false;
        ModelFileDescription otherMyClass = (ModelFileDescription)other;
        boolean returnedValue = true;

        returnedValue &= this.hasModel_ == otherMyClass.hasModel_;

                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.modelLoaderClass_, otherMyClass.modelLoaderClass_);
                
        returnedValue &= this.resourceDirectories_.equals(otherMyClass.resourceDirectories_);
                
        returnedValue &= this.modelFileSize_ == otherMyClass.modelFileSize_;

                
        returnedValue &= this.hasResourceZip_ == otherMyClass.hasResourceZip_;

                
        returnedValue &= this.resourceZipSize_ == otherMyClass.resourceZipSize_;

                

        return returnedValue;
    }
    
     @Override
    public java.lang.String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("ModelFileDescription {");
        builder.append("hasModel=");
        builder.append(this.hasModel_);

                builder.append(", ");
        builder.append("name=");
        builder.append(this.name_);

                builder.append(", ");
        builder.append("modelLoaderClass=");
        builder.append(this.modelLoaderClass_);

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
    private java.lang.StringBuilder name_; 
    private java.lang.StringBuilder modelLoaderClass_; 
    private us.ihmc.idl.IDLSequence.StringBuilderHolder  resourceDirectories_; 
    private int modelFileSize_; 
    private boolean hasResourceZip_; 
    private int resourceZipSize_; 

}