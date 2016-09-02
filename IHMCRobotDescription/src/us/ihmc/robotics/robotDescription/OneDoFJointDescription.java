package us.ihmc.robotics.robotDescription;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.Axis;

public class OneDoFJointDescription extends JointDescription
{
   private boolean containsLimitStops;
   private double qMin, qMax, kLimit, bLimit;

   private double maxTorqueLimit = Double.POSITIVE_INFINITY;
   private double minTorqueLimit = Double.NEGATIVE_INFINITY;

   private double velocityLimit = Double.POSITIVE_INFINITY;
   private double velocityDamping = 0.0;

   private double damping = 0.0;
   private double stiction = 0.0;

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

   public Vector3d getJointAxis()
   {
      return jointAxis;
   }

   public boolean containsLimitStops()
   {
      return containsLimitStops;
   }

   public double[] getLimitStopParameters()
   {
      return new double[]{qMin, qMax, kLimit, bLimit};
   }

   public void setTorqueLimits(double torqueLimit)
   {
      this.maxTorqueLimit = Math.abs(torqueLimit);
      this.minTorqueLimit = -Math.abs(torqueLimit);
   }

}
