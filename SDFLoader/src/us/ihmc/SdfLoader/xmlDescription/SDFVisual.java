package us.ihmc.SdfLoader.xmlDescription;

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

   @XmlElement(name="laser_retro")
   public void setLaserRetro(String laserRetro)
   {
      this.laserRetro = laserRetro;
   }

   public String getTransparency()
   {
      return transparency;
   }

   @XmlElement(name="transparency")
   public void setTransparency(String transparency)
   {
      this.transparency = transparency;
   }

   public String getPose()
   {
      return pose;
   }

   @XmlElement(name="pose")
   public void setPose(String pose)
   {
      this.pose = pose;
   }

   public SDFGeometry getGeometry()
   {
      return geometry;
   }

   @XmlElement(name="geometry")
   public void setGeometry(SDFGeometry geometry)
   {
      this.geometry = geometry;
   }

}