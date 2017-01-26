package us.ihmc.modelFileLoaders.SdfLoader.xmlDescription;

import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;

public class SDFLink
{
   private String name;
   private String pose;
   private Inertial inertial;
   private List<SDFVisual> visuals;
   private List<SDFSensor> sensors;
   private List<Collision> collisions;

   public String getName()
   {
      return name;
   }

   @XmlAttribute(name = "name")
   public void setName(String name)
   {
      this.name = name;
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

   public Inertial getInertial()
   {
      return inertial;
   }

   @XmlElement(name = "inertial")
   public void setInertial(Inertial inertial)
   {
      this.inertial = inertial;
   }

   public List<SDFVisual> getVisuals()
   {
      return visuals;
   }

   @XmlElement(name = "visual")
   public void setVisuals(List<SDFVisual> visual)
   {
      this.visuals = visual;
   }

   public List<Collision> getCollisions()
   {
      return collisions;
   }

   @XmlElement(name = "collision")
   public void setCollisions(List<Collision> collision)
   {
      this.collisions = collision;
   }

   public List<SDFSensor> getSensors()
   {
      return sensors;
   }

   @XmlElement(name = "sensor")
   public void setSensors(List<SDFSensor> sensors)
   {
      this.sensors = sensors;
   }
   
   public static class Inertial
   {
      private String mass;
      private String pose;
      private SDFInertia inertia;

      public String getMass()
      {
         return mass;
      }

      @XmlElement(name = "mass")
      public void setMass(String mass)
      {
         this.mass = mass;
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

      public SDFInertia getInertia()
      {
         return inertia;
      }

      @XmlElement(name = "inertia")
      public void setInertia(SDFInertia inertia)
      {
         this.inertia = inertia;
      }
   }

   public String toString()
   {
      return name;
   }

   
}