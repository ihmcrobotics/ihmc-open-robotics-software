package us.ihmc.SdfLoader.xmlDescription;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;

public class SDFSensor
{
   private String name;
   private String type;
   private String updateRate;
   private String pose;
   private Camera camera;
   private Ray ray;

   private Plugin plugin;

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

   public Camera getCamera()
   {
      return camera;
   }

   @XmlElement(name = "camera")
   public void setCamera(Camera camera)
   {
      this.camera = camera;
   }

   @XmlElement(name = "ray")
   public void setRay(Ray ray)
   {
      this.ray = ray;
   }

   @XmlElement(name = "plugin")
   public void setPlugin(Plugin plugin)
   {
      this.plugin = plugin;
   }

   public Ray getRay()
   {
      return ray;
   }

   public Plugin getPlugin()
   {
      return plugin;
   }

   public static class Plugin
   {
      /*
       * <plugin name='gazebo_ros_head_hokuyo_controller' filename='libgazebo_ros_laser.so'>
       *    <gaussianNoise>0.005</gaussianNoise>
       *    <alwaysOn>1</alwaysOn>
       *    <updateRate>40</updateRate>
       *    <topicName>/multisense_sl/scan</topicName>
       *    <frameName>head_hokuyo_frame</frameName>
       * </plugin>
       */
      private String gaussianNoise;
      private String alwaysOn;
      private String updateRate;
      private String topicName;
      private String frameName;
      private String name;
      private String filename;

      public String getGaussianNoise()
      {
         return gaussianNoise;
      }

      @XmlElement(name = "gaussianNoise")
      public void setGaussianNoise(String gaussianNoise)
      {
         this.gaussianNoise = gaussianNoise;
      }

      public String getAlwaysOn()
      {
         return alwaysOn;
      }

      @XmlElement(name = "alwaysOn")
      public void setAlwaysOn(String alwaysOn)
      {
         this.alwaysOn = alwaysOn;
      }

      public String getUpdateRate()
      {
         return updateRate;
      }

      @XmlElement(name = "updateRate")
      public void setUpdateRate(String updateRate)
      {
         this.updateRate = updateRate;
      }

      public String getTopicName()
      {
         return topicName;
      }

      @XmlElement(name = "topicName")
      public void setTopicName(String topicName)
      {
         this.topicName = topicName;
      }

      public String getFrameName()
      {
         return frameName;
      }

      @XmlElement(name = "frameName")
      public void setFrameName(String frameName)
      {
         this.frameName = frameName;
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

      public String getFilename()
      {
         return filename;
      }

      @XmlElement(name = "filename")
      public void setFilename(String filename)
      {
         this.filename = filename;
      }

   }


   public static class Ray
   {
      /*
       *  <ray>
       *    <scan>
       *      <horizontal>
       *        <samples>720</samples>
       *        <resolution>1.000000</resolution>
       *        <min_angle>-1.570796</min_angle>
       *        <max_angle>1.570796</max_angle>
       *      </horizontal>
       *    </scan>
       *    <range>
       *      <min>0.100000</min>
       *      <max>30.000000</max>
       *      <resolution>0.010000</resolution>
       *    </range>
       *  </ray>
       */
      private Range range;

      private Scan scan;

      @XmlElement(name = "range")
      public void setRange(Range range)
      {
         this.range = range;
      }

      @XmlElement(name = "scan")
      public void setScan(Scan scan)
      {
         this.scan = scan;
      }

      public Range getRange()
      {
         return range;
      }

      public Scan getScan()
      {
         return scan;
      }

      public static class Range
      {
         private String min;

         private String max;

         private String resolution;

         @XmlElement(name = "min")
         public void setMin(String min)
         {
            this.min = min;
         }

         @XmlElement(name = "max")
         public void setMax(String max)
         {
            this.max = max;
         }

         @XmlElement(name = "resolution")
         public void setResolution(String resolution)
         {
            this.resolution = resolution;
         }

         public String getMin()
         {
            return min;
         }

         public String getMax()
         {
            return max;
         }

         public String getResolution()
         {
            return resolution;
         }
      }


      public static class Scan
      {
         private HorizontalScan horizontal;

         @XmlElement(name = "horizontal")
         public void setHorizontal(HorizontalScan horizontal)
         {
            this.horizontal = horizontal;
         }

         public HorizontalScan getHorizontal()
         {
            return horizontal;
         }

         public static class HorizontalScan
         {
            private String samples;

            private String resolution;

            private String minAngle;

            private String maxAngle;

            @XmlElement(name = "samples")
            public void setSamples(String samples)
            {
               this.samples = samples;
            }

            @XmlElement(name = "resolution")
            public void setResolution(String resolution)
            {
               this.resolution = resolution;
            }

            @XmlElement(name = "min_angle")
            public void setMinAngle(String minAngle)
            {
               this.minAngle = minAngle;
            }

            @XmlElement(name = "max_angle")
            public void setMaxAngle(String maxAngle)
            {
               this.maxAngle = maxAngle;
            }

            public String getSamples()
            {
               return samples;
            }

            public String getResolution()
            {
               return resolution;
            }

            public String getMinAngle()
            {
               return minAngle;
            }

            public String getMaxAngle()
            {
               return maxAngle;
            }

         }

      }
   }


   public static class Camera
   {
      private String pose;
      private String horizontalFov;
      private SensorImage image;
      private Clip clip;

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

      public SensorImage getImage()
      {
         return image;
      }

      @XmlElement(name = "image")
      public void setImage(SensorImage image)
      {
         this.image = image;
      }

      public Clip getClip()
      {
         return clip;
      }

      @XmlElement(name = "clip")
      public void setClip(Clip clip)
      {
         this.clip = clip;
      }

      public static class SensorImage
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


      public static class Clip
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
