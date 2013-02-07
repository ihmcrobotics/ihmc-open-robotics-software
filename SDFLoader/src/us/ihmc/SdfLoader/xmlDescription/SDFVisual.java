package us.ihmc.SdfLoader.xmlDescription;

import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;

public class SDFVisual
{
   private String name;
   private String castShadows;
   private String laserRetro;
   private String transparency;
   private String pose;
   private SDFGeometry geometry;
   private SDFMaterial material;

   public String getName()
   {
      return name;
   }

   @XmlAttribute(name = "name")
   public void setName(String name)
   {
      this.name = name;
   }

   public String getCastShadows()
   {
      return castShadows;
   }

   @XmlElement(name = "cast_shadows")
   public void setCastShadows(String castShadows)
   {
      this.castShadows = castShadows;
   }

   public String getLaserRetro()
   {
      return laserRetro;
   }

   @XmlElement(name = "laser_retro")
   public void setLaserRetro(String laserRetro)
   {
      this.laserRetro = laserRetro;
   }

   public String getTransparency()
   {
      return transparency;
   }

   @XmlElement(name = "transparency")
   public void setTransparency(String transparency)
   {
      this.transparency = transparency;
   }

   public String getPose()
   {
      return pose;
   }

   @XmlElement(name = "pose")
   public void setPose(String pose)
   {
      this.pose = pose;
   }

   public SDFGeometry getGeometry()
   {
      return geometry;
   }

   @XmlElement(name = "geometry")
   public void setGeometry(SDFGeometry geometry)
   {
      this.geometry = geometry;
   }

   public SDFMaterial getMaterial()
   {
      return material;
   }

   @XmlElement(name = "material")
   public void setMaterial(SDFMaterial material)
   {
      this.material = material;
   }

   public static class SDFMaterial
   {
      private SDFScript script;


      public SDFScript getScript()
      {
         return script;
      }

      @XmlElement(name = "script")
      public void setScript(SDFScript script)
      {
         this.script = script;
      }
      
      public static class SDFScript
      {
         private List<String> uri;
         private String name;

         public List<String> getUri()
         {
            return uri;
         }

         @XmlElement(name = "uri")
         public void setUri(List<String> uri)
         {
            this.uri = uri;
         }

         public String getName()
         {
            return name;
         }

         @XmlElement(name = "name")
         public void setName(String name)
         {
            this.name = name;
         }

      }

   }

}