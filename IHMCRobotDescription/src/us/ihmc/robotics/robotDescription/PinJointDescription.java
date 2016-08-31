package us.ihmc.robotics.robotDescription;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.Axis;

public class PinJointDescription extends JointDescription
{
   private boolean containsLimitStops;
   private double qMin, qMax, kLimit, bLimit;

   private final Vector3d jointAxis = new Vector3d();

   public PinJointDescription(String name, Vector3d offsetFromParentJoint, Axis jointAxis)
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

}
