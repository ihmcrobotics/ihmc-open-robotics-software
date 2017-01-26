package us.ihmc.modelFileLoaders.SdfLoader.xmlDescription;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;

public class SDFJoint
{
   private String name;
   private String type;

   private String child;
   private String parent;
   private String pose;

   private String threadPitch;

   private Axis axis;
   private Axis axis2;

   @XmlAttribute(name = "name")
   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   @XmlAttribute(name = "type")
   public String getType()
   {
      return type;
   }

   public void setType(String type)
   {
      this.type = type;
   }

   public String getParent()
   {
      return parent;
   }

   @XmlElement(name = "parent")
   public void setParent(String parent)
   {
      this.parent = parent;
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

   public String getThreadPitch()
   {
      return threadPitch;
   }

   @XmlElement(name = "thread_pitch")
   public void setThreadPitch(String threadPitch)
   {
      this.threadPitch = threadPitch;
   }

   public Axis getAxis()
   {
      return axis;
   }

   @XmlElement(name = "axis")
   public void setAxis(Axis axis)
   {
      this.axis = axis;
   }

   public Axis getAxis2()
   {
      return axis2;
   }

   @XmlElement(name = "axis2")
   public void setAxis2(Axis axis2)
   {
      this.axis2 = axis2;
   }

   public static class Axis
   {
      private String xyz;

      private Dynamics dynamics;
      private Limit limit;

      public String getXyz()
      {
         return xyz;
      }

      @XmlElement(name = "xyz")
      public void setXyz(String xyz)
      {
         this.xyz = xyz;
      }

      public Dynamics getDynamics()
      {
         return dynamics;
      }

      @XmlElement(name = "dynamics")
      public void setDynamics(Dynamics dynamics)
      {
         this.dynamics = dynamics;
      }

      public Limit getLimit()
      {
         return limit;
      }

      @XmlElement(name = "limit")
      public void setLimit(Limit limit)
      {
         this.limit = limit;
      }

      public static class Dynamics
      {
         private String damping;
         private String friction;

         public String getDamping()
         {
            return damping;
         }

         @XmlElement(name = "damping")
         public void setDamping(String damping)
         {
            this.damping = damping;
         }

         public String getFriction()
         {
            return friction;
         }

         @XmlElement(name = "friction")
         public void setFriction(String friction)
         {
            this.friction = friction;
         }

      }

      public static class Limit
      {
         private String lower;
         private String upper;

         private String effort;
         private String velocity;

         public String getLower()
         {
            return lower;
         }

         @XmlElement(name = "lower")
         public void setLower(String lower)
         {
            this.lower = lower;
         }

         public String getUpper()
         {
            return upper;
         }

         @XmlElement(name = "upper")
         public void setUpper(String upper)
         {
            this.upper = upper;
         }

         public String getEffort()
         {
            return effort;
         }

         @XmlElement(name = "effort")
         public void setEffort(String effort)
         {
            this.effort = effort;
         }

         public String getVelocity()
         {
            return velocity;
         }

         @XmlElement(name = "velocity")
         public void setVelocity(String velocity)
         {
            this.velocity = velocity;
         }
      }
   }

   public String toString()
   {
      return name;
   }

   public String getChild()
   {
      return child;
   }

   @XmlElement(name = "child")
   public void setChild(String child)
   {
      this.child = child;
   }
}