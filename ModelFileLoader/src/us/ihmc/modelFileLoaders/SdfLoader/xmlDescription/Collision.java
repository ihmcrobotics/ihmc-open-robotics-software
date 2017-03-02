package us.ihmc.modelFileLoaders.SdfLoader.xmlDescription;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;

import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFVisual.SDFMaterial;

public class Collision implements AbstractSDFMesh
{
   private String name;
   private String pose;
   private Surface surface;
   private SDFGeometry geometry;

   public static class Surface
   {
      private Contact contact;

      public static class Contact
      {
         private Ode ode;

         public static class Ode
         {
            private String kp;
            private String kd;
            private String maxVel;

            public String getKp()
            {
               return kp;
            }

            @XmlElement(name = "kp")
            public void setKp(String kp)
            {
               this.kp = kp;
            }

            public String getKd()
            {
               return kd;
            }

            @XmlElement(name = "kd")
            public void setKd(String kd)
            {
               this.kd = kd;
            }

            public String getMaxVel()
            {
               return maxVel;
            }

            @XmlElement(name = "max_vel")
            public void setMaxVel(String maxVel)
            {
               this.maxVel = maxVel;
            }

         }

         public Ode getOde()
         {
            return ode;
         }

         @XmlElement(name = "ode")
         public void setOde(Ode ode)
         {
            this.ode = ode;
         }
      }

      public Contact getContact()
      {
         return contact;
      }

      @XmlElement(name = "contact")
      public void setContact(Contact contact)
      {
         this.contact = contact;
      }
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
   
   public String getName()
   {
      return name;
   }

   @XmlAttribute(name = "name")
   public void setName(String name)
   {
      this.name = name;
   }

   public Surface getSurface()
   {
      return surface;
   }

   @XmlElement(name = "surface")
   public void setSurface(Surface surface)
   {
      this.surface = surface;
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
      return null;
   }

   @Override
   public String getTransparency()
   {
      return null;
   }
}
