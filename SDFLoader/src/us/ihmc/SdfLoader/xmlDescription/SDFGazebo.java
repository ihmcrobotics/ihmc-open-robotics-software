package us.ihmc.SdfLoader.xmlDescription;


import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name="gazebo")
public class SDFGazebo
{
   private String version;
   
   private SDFWorld world;
   
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


   public String toString()
   {
      return world.toString();
   }


}
