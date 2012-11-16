package us.ihmc.SdfLoader.xmlDescription;

import java.util.List;

import javax.xml.bind.annotation.XmlElement;

public class SDFWorld
{
   private List<SDFModel> models; 
   
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
      return models.toString();
   }
}