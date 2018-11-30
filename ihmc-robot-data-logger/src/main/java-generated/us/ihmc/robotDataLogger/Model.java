package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class Model extends Packet<Model> implements Settable<Model>, EpsilonComparable<Model>
{
   public java.lang.StringBuilder loader_;
   // Loader class
   public java.lang.StringBuilder path_;
   // Path
   public java.lang.StringBuilder name_;
   // Model file name
   public java.lang.StringBuilder resourceBundle_;
   // Resource bundle zip file name
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  resourceDirectoriesList_;

   public Model()
   {
      loader_ = new java.lang.StringBuilder(255);
      path_ = new java.lang.StringBuilder(255);
      name_ = new java.lang.StringBuilder(255);
      resourceBundle_ = new java.lang.StringBuilder(255);
      resourceDirectoriesList_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (255, "type_d");
   }

   public Model(Model other)
   {
      this();
      set(other);
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

   public void setLoader(java.lang.String loader)
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

   // Loader class
   public void setPath(java.lang.String path)
   {
      path_.setLength(0);
      path_.append(path);
   }

   // Loader class
   public java.lang.String getPathAsString()
   {
      return getPath().toString();
   }
   // Loader class
   public java.lang.StringBuilder getPath()
   {
      return path_;
   }

   // Path
   public void setName(java.lang.String name)
   {
      name_.setLength(0);
      name_.append(name);
   }

   // Path
   public java.lang.String getNameAsString()
   {
      return getName().toString();
   }
   // Path
   public java.lang.StringBuilder getName()
   {
      return name_;
   }

   // Model file name
   public void setResourceBundle(java.lang.String resourceBundle)
   {
      resourceBundle_.setLength(0);
      resourceBundle_.append(resourceBundle);
   }

   // Model file name
   public java.lang.String getResourceBundleAsString()
   {
      return getResourceBundle().toString();
   }
   // Model file name
   public java.lang.StringBuilder getResourceBundle()
   {
      return resourceBundle_;
   }


   // Resource bundle zip file name
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getResourceDirectoriesList()
   {
      return resourceDirectoriesList_;
   }


   public static Supplier<ModelPubSubType> getPubSubType()
   {
      return ModelPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ModelPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Model other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.loader_, other.loader_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.path_, other.path_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.name_, other.name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.resourceBundle_, other.resourceBundle_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.resourceDirectoriesList_, other.resourceDirectoriesList_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Model)) return false;

      Model otherMyClass = (Model) other;

      if (!us.ihmc.idl.IDLTools.equals(this.loader_, otherMyClass.loader_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.path_, otherMyClass.path_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.resourceBundle_, otherMyClass.resourceBundle_)) return false;

      if (!this.resourceDirectoriesList_.equals(otherMyClass.resourceDirectoriesList_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Model {");
      builder.append("loader=");
      builder.append(this.loader_);      builder.append(", ");
      builder.append("path=");
      builder.append(this.path_);      builder.append(", ");
      builder.append("name=");
      builder.append(this.name_);      builder.append(", ");
      builder.append("resourceBundle=");
      builder.append(this.resourceBundle_);      builder.append(", ");
      builder.append("resourceDirectoriesList=");
      builder.append(this.resourceDirectoriesList_);
      builder.append("}");
      return builder.toString();
   }
}
