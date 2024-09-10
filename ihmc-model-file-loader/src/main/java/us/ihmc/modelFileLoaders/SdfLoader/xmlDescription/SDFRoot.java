package us.ihmc.modelFileLoaders.SdfLoader.xmlDescription;


import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name="sdf")
public class SDFRoot
{
   private String version;
   
   private SDFWorld world;
   
   private List<SDFModel> models; 

   
   public String getVersion()
   {
      return version;
   }

   @XmlAttribute(name="version")
   public void setVersion(String version)
   {
      this.version = version;
   }
   
   
   public SDFWorld getWorld()
   {
      return world;
   }

   @XmlElement(name="world")
   public void setWorld(SDFWorld world)
   {
      this.world = world;
   }
   
   public List<SDFModel> getModels()
   {
      return models;
   }

   @XmlElement(name="model")
   public void setModels(List<SDFModel> models)
   {
      this.models = models;
   }
   
   public String toString()
   {
      return world.toString();
   }


}
