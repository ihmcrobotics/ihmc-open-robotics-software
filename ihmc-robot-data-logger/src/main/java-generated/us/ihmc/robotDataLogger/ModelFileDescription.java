package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class ModelFileDescription extends Packet<ModelFileDescription> implements Settable<ModelFileDescription>, EpsilonComparable<ModelFileDescription>
{
   public boolean has_model_;
   public java.lang.StringBuilder name_;
   public java.lang.StringBuilder modelLoaderClass_;
   public us.ihmc.idl.IDLSequence.StringBuilderHolder resourceDirectories_;
   public int modelFileSize_;
   public boolean hasResourceZip_;
   public int resourceZipSize_;

   public ModelFileDescription()
   {

      name_ = new java.lang.StringBuilder(255);

      modelLoaderClass_ = new java.lang.StringBuilder(255);

      resourceDirectories_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder(255, "type_d");
   }

   public ModelFileDescription(ModelFileDescription other)
   {
      set(other);
   }

   public void set(ModelFileDescription other)
   {
      has_model_ = other.has_model_;

      name_.setLength(0);
      name_.append(other.name_);

      modelLoaderClass_.setLength(0);
      modelLoaderClass_.append(other.modelLoaderClass_);

      resourceDirectories_.set(other.resourceDirectories_);
      modelFileSize_ = other.modelFileSize_;

      hasResourceZip_ = other.hasResourceZip_;

      resourceZipSize_ = other.resourceZipSize_;
   }

   public boolean getHasModel()
   {
      return has_model_;
   }

   public void setHasModel(boolean has_model)
   {
      has_model_ = has_model;
   }

   public java.lang.String getNameAsString()
   {
      return getName().toString();
   }

   public java.lang.StringBuilder getName()
   {
      return name_;
   }

   public void setName(java.lang.String name)
   {
      name_.setLength(0);
      name_.append(name);
   }

   public java.lang.String getModelLoaderClassAsString()
   {
      return getModelLoaderClass().toString();
   }

   public java.lang.StringBuilder getModelLoaderClass()
   {
      return modelLoaderClass_;
   }

   public void setModelLoaderClass(java.lang.String modelLoaderClass)
   {
      modelLoaderClass_.setLength(0);
      modelLoaderClass_.append(modelLoaderClass);
   }

   public us.ihmc.idl.IDLSequence.StringBuilderHolder getResourceDirectories()
   {
      return resourceDirectories_;
   }

   public int getModelFileSize()
   {
      return modelFileSize_;
   }

   public void setModelFileSize(int modelFileSize)
   {
      modelFileSize_ = modelFileSize;
   }

   public boolean getHasResourceZip()
   {
      return hasResourceZip_;
   }

   public void setHasResourceZip(boolean hasResourceZip)
   {
      hasResourceZip_ = hasResourceZip;
   }

   public int getResourceZipSize()
   {
      return resourceZipSize_;
   }

   public void setResourceZipSize(int resourceZipSize)
   {
      resourceZipSize_ = resourceZipSize;
   }

   @Override
   public boolean epsilonEquals(ModelFileDescription other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_model_, other.has_model_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.name_, other.name_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.modelLoaderClass_, other.modelLoaderClass_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.resourceDirectories_, other.resourceDirectories_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.modelFileSize_, other.modelFileSize_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.hasResourceZip_, other.hasResourceZip_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.resourceZipSize_, other.resourceZipSize_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof ModelFileDescription))
         return false;

      ModelFileDescription otherMyClass = (ModelFileDescription) other;

      if (this.has_model_ != otherMyClass.has_model_)
         return false;

      if (!us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_))
         return false;

      if (!us.ihmc.idl.IDLTools.equals(this.modelLoaderClass_, otherMyClass.modelLoaderClass_))
         return false;

      if (!this.resourceDirectories_.equals(otherMyClass.resourceDirectories_))
         return false;

      if (this.modelFileSize_ != otherMyClass.modelFileSize_)
         return false;

      if (this.hasResourceZip_ != otherMyClass.hasResourceZip_)
         return false;

      if (this.resourceZipSize_ != otherMyClass.resourceZipSize_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ModelFileDescription {");
      builder.append("has_model=");
      builder.append(this.has_model_);

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
}