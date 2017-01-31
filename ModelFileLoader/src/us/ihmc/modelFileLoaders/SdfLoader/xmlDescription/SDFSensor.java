package us.ihmc.modelFileLoaders.SdfLoader.xmlDescription;

import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;

public class SDFSensor
{
   private String name;
   private String type;
   private String updateRate;
   private String pose;
   private List<Camera> camera;
   private Ray ray;
   private IMU imu;

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

   public List<Camera> getCamera()
   {
      return camera;
   }

   @XmlElement(name = "camera")
   public void setCamera(List<Camera> camera)
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

   public IMU getImu()
   {
      return imu;
   }

   @XmlElement(name = "imu")
   public void setImu(IMU imu)
   {
      this.imu = imu;
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
      private String pose;

      private Range range;

      private Scan scan;

      private Noise noise;

      @XmlElement(name = "pose")
      public void setPose(String pose)
      {
         this.pose = pose;
      }

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

      public String getPose()
      {
         return pose;
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
         private VerticalScan vertical;

         @XmlElement(name = "horizontal")
         public void setHorizontal(HorizontalScan horizontal)
         {
            this.horizontal = horizontal;
         }

         public HorizontalScan getHorizontal()
         {
            return horizontal;
         }

         @XmlElement(name = "vertical")
         public VerticalScan getVertical()
         {
            return vertical;
         }

         public void setVertical(VerticalScan vertical)
         {
            this.vertical = vertical;
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
         
         public static class VerticalScan
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
      private String name;
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

      @XmlAttribute(name = "name")
      public String getName()
      {
         return name;
      }

      public void setName(String name)
      {
         this.name = name;
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

   public static class IMU
   {
      //      <imu>
      //      <noise>
      //        <type>gaussian</type>
      //        <rate>
      //          <mean>0.000000</mean>
      //          <stddev>0.000200</stddev>
      //          <bias_mean>0.000008</bias_mean>
      //          <bias_stddev>0.000001</bias_stddev>
      //        </rate>
      //        <accel>
      //          <mean>0.000000</mean>
      //          <stddev>0.017000</stddev>
      //          <bias_mean>0.100000</bias_mean>
      //          <bias_stddev>0.001000</bias_stddev>
      //        </accel>
      //      </noise>
      //    </imu>

      private IMUNoise noise;

      public static class IMUNoise
      {
         private String type;
         private NoiseParameters rate;
         private NoiseParameters accel;

         public String getType()
         {
            return type;
         }

         public NoiseParameters getRate()
         {
            return rate;
         }

         public NoiseParameters getAccel()
         {
            return accel;
         }

         @XmlElement(name = "type")
         public void setType(String type)
         {
            this.type = type;
         }

         @XmlElement(name = "rate")
         public void setRate(NoiseParameters rate)
         {
            this.rate = rate;
         }

         @XmlElement(name = "accel")
         public void setAccel(NoiseParameters accel)
         {
            this.accel = accel;
         }

         public static class NoiseParameters
         {
            private String mean;
            private String stddev;
            private String bias_mean;
            private String bias_stddev;

            public String getMean()
            {
               return mean;
            }

            public String getStddev()
            {
               return stddev;
            }

            public String getBias_mean()
            {
               return bias_mean;
            }

            public String getBias_stddev()
            {
               return bias_stddev;
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

            @XmlElement(name = "bias_mean")
            public void setBias_mean(String bias_mean)
            {
               this.bias_mean = bias_mean;
            }

            @XmlElement(name = "bias_stddev")
            public void setBias_stddev(String bias_stddev)
            {
               this.bias_stddev = bias_stddev;
            }
         }

      }

      public IMUNoise getNoise()
      {
         return noise;
      }

      @XmlElement(name = "noise")
      public void setNoise(IMUNoise noise)
      {
         this.noise = noise;
      }
   }
}
