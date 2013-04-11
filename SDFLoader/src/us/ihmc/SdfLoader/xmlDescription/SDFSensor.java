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


   public Ray getRay()
   {
      return ray;
   }



   public static class Ray
   {
      /*
       * <ray> <scan> <horizontal> <samples>720</samples>
       * <resolution>1.000000</resolution> <min_angle>-1.570796</min_angle>
       * <max_angle>1.570796</max_angle> </horizontal> </scan> <range>
       * <min>0.100000</min> <max>30.000000</max>
       * <resolution>0.010000</resolution> </range> </ray>
       */
      private Range range;

      private Scan scan;

      private Noise noise;

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

      public Noise getNoise()
      {
         return noise;
      }

      @XmlElement(name = "noise")
      public void setNoise(Noise noise)
      {
         this.noise = noise;
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
      
      public static class Noise
      {
         //            <noise>
         //            <type>gaussian</type>
         //            <mean>0.000000</mean>
         //            <stddev>0.010000</stddev>
         //          </noise>

         private String type;
         private String mean;
         private String stddev;

         @XmlElement(name = "type")
         public void setType(String type)
         {
            this.type = type;
         }

         @XmlElement(name = "mean")
         public void setMean(String mean)
         {
            this.mean = mean;
         }

         @XmlElement(name = "stddev")
         public void setStddev(String stddev)
         {
            this.stddev = stddev;
         }

         public String getType()
         {
            return type;
         }

         public String getMean()
         {
            return mean;
         }

         public String getStddev()
         {
            return stddev;
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
