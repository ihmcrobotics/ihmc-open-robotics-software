package us.ihmc.robotics.robotDescription;

import java.util.List;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.Axis;

public class OneDoFJointDescription extends JointDescription
{
   private boolean containsLimitStops;
   private double qMin = Double.NEGATIVE_INFINITY;
   private double qMax = Double.POSITIVE_INFINITY;
   private double kLimit, bLimit;

   private double effortLimit = Double.POSITIVE_INFINITY;

   private double velocityLimit = Double.POSITIVE_INFINITY;
   private double velocityDamping;

   private double damping;
   private double stiction;

   private final Vector3d jointAxis = new Vector3d();

   public OneDoFJointDescription(String name, Vector3d offsetFromParentJoint, Axis jointAxis)
   {
      super(name, offsetFromParentJoint);

      switch (jointAxis)
      {
      case X:
      {
         this.jointAxis.set(1.0, 0.0, 0.0);
         break;
      }
      case Y:
      {
         this.jointAxis.set(0.0, 1.0, 0.0);
         break;
      }
      case Z:
      {
         this.jointAxis.set(0.0, 0.0, 1.0);
         break;
      }
      }
   }

   public OneDoFJointDescription(String name, Vector3d offset, Vector3d jointAxis)
   {
      super(name, offset);
      this.jointAxis.set(jointAxis);
   }

   public void setVelocityLimits(double velocityLimit, double velocityDamping)
   {
      this.velocityLimit = velocityLimit;
      this.velocityDamping = velocityDamping;
   }

   public void setDamping(double damping)
   {
      this.damping = damping;
   }

   public double getDamping()
   {
      return damping;
   }

   public double getStiction()
   {
      return stiction;
   }

   public double getVelocityLimit()
   {
      return velocityLimit;
   }

   public double getVelocityDamping()
   {
      return velocityDamping;
   }

   public void setStiction(double stiction)
   {
      this.stiction = stiction;
   }

   public void setLimitStops(double qMin, double qMax, double kLimit, double bLimit)
   {
      this.containsLimitStops = true;
      this.qMin = qMin;
      this.qMax = qMax;
      this.kLimit = kLimit;
      this.bLimit = bLimit;
   }

   public void getJointAxis(Vector3d jointAxisToPack)
   {
      jointAxisToPack.set(jointAxis);
   }

   public boolean containsLimitStops()
   {
      return containsLimitStops;
   }

   public double[] getLimitStopParameters()
   {
      return new double[] { qMin, qMax, kLimit, bLimit };
   }

   public double getLowerLimit()
   {
      return qMin;
   }

   public double getUpperLimit()
   {
      return qMax;
   }

   public void setEffortLimit(double effortLimit)
   {
      this.effortLimit = effortLimit;
   }

   public double getEffortLimit()
   {
      return effortLimit;
   }
   
   @Override
   public void scale(double factor, double massScalePower, List<String> ignoreInertiaScaleJointList)
   {
      double massScale = Math.pow(factor, massScalePower);
      double dampingScale = Math.pow(factor, massScalePower + 2); // Joint acceleration is related to inertia.
      this.damping = massScale * this.damping;
      
      this.kLimit = massScale * this.kLimit;
      this.bLimit = massScale * this.bLimit;
      
      this.velocityDamping = massScale * this.velocityDamping;
      
      super.scale(factor, massScalePower, ignoreInertiaScaleJointList);
   }
}
