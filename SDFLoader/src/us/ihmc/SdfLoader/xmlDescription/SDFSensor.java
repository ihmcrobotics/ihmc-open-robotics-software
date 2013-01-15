package us.ihmc.SdfLoader.xmlDescription;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;

public class SDFSensor
{
   private String name;
   private String type;
   private String updateRate;
   private String pose;
   private SDFCamera camera;

   public String getName()
   {
      return name;
   }

   @XmlAttribute(name = "name")
   public void setName(String name)
   {
      this.name = name;
   }

   public String getType()
   {
      return type;
   }

   @XmlAttribute(name = "type")
   public void setType(String type)
   {
      this.type = type;
   }

   public String getUpdateRate()
   {
      return updateRate;
   }

   @XmlElement(name = "update_rate")
   public void setUpdateRate(String updateRate)
   {
      this.updateRate = updateRate;
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

   public SDFCamera getCamera()
   {
      return camera;
   }

   @XmlElement(name = "camera")
   public void setCamera(SDFCamera camera)
   {
      this.camera = camera;
   }

   public static class SDFCamera
   {
      private String pose;
      private String horizontalFov;
      private SDFImage image;
      private SDFClip clip;

      public String getPose()
      {
         return pose;
      }

      @XmlElement(name = "pose")
      public void setPose(String pose)
      {
         this.pose = pose;
      }

      public String getHorizontalFov()
      {
         return horizontalFov;
      }

      @XmlElement(name = "horizontal_fov")
      public void setHorizontalFov(String horizontalFov)
      {
         this.horizontalFov = horizontalFov;
      }

      public SDFImage getImage()
      {
         return image;
      }

      @XmlElement(name = "image")
      public void setImage(SDFImage image)
      {
         this.image = image;
      }

      public SDFClip getClip()
      {
         return clip;
      }

      @XmlElement(name = "clip")
      public void setClip(SDFClip clip)
      {
         this.clip = clip;
      }

      public static class SDFImage
      {
         private String width;
         private String height;
         private String format;

         public String getWidth()
         {
            return width;
         }

         @XmlElement(name = "width")
         public void setWidth(String width)
         {
            this.width = width;
         }

         public String getHeight()
         {
            return height;
         }

         @XmlElement(name = "height")
         public void setHeight(String height)
         {
            this.height = height;
         }

         public String getFormat()
         {
            return format;
         }

         @XmlElement(name = "format")
         public void setFormat(String format)
         {
            this.format = format;
         }
      }

      public static class SDFClip
      {
         private String near;
         private String far;

         public String getNear()
         {
            return near;
         }

         @XmlElement(name = "near")
         public void setNear(String near)
         {
            this.near = near;
         }

         public String getFar()
         {
            return far;
         }

         @XmlElement(name = "far")
         public void setFar(String far)
         {
            this.far = far;
         }
      }
   }
}
