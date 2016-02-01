package us.ihmc.SdfLoader.xmlDescription;

import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;

public class SDFWorld
{
   private List<SDFModel> models;
   private List<Road> roads;

   public List<SDFModel> getModels()
   {
      return models;
   }

   @XmlElement(name = "model")
   public void setModels(List<SDFModel> models)
   {
      this.models = models;
   }

   public List<Road> getRoads()
   {
      return roads;
   }

   @XmlElement(name = "road")
   public void setRoads(List<Road> roads)
   {
      this.roads = roads;
   }

   public String toString()
   {
      return models.toString();
   }

   public static class Road
   {
      private String name;
      private String width;
      private List<String> points;

      public String getName()
      {
         return name;
      }

      @XmlAttribute(name = "name")
      public void setName(String name)
      {
         this.name = name;
      }

      public String getWidth()
      {
         return width;
      }

      @XmlElement(name = "width")
      public void setWidth(String width)
      {
         this.width = width;
      }

      public List<String> getPoints()
      {
         return points;
      }

      @XmlElement(name = "point")
      public void setPoints(List<String> points)
      {
         this.points = points;
      }

   }
}