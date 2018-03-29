package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import us.ihmc.euclid.tuple3D.Vector3D;

public class CentroidalMotionPlannerParameters
{
   private Vector3D gravity = new Vector3D();
   private Vector3D maximumForce = new Vector3D();
   private Vector3D minimumForce = new Vector3D();
   private Vector3D maximumForceRate = new Vector3D();
   private Vector3D minimumForceRate = new Vector3D();
   private double robotMass;
   private double forceRegulizationWeight;
   private double dForceRegularizationWeight;
   private double nominalIxx;
   private double nominalIyy;
   private double nominalIzz;
   private double deltaTMin;

   public double getNominalIxx()
   {
      return nominalIxx;
   }

   public void setNominalIxx(double nominalIxx)
   {
      this.nominalIxx = nominalIxx;
   }

   public double getNominalIyy()
   {
      return nominalIyy;
   }

   public void setNominalIyy(double nominalIyy)
   {
      this.nominalIyy = nominalIyy;
   }

   public double getNominalIzz()
   {
      return nominalIzz;
   }

   public void setNominalIzz(double nominalIzz)
   {
      this.nominalIzz = nominalIzz;
   }

   public double getDeltaTMin()
   {
      return deltaTMin;
   }

   public void setDeltaTMin(double deltaTMin)
   {
      this.deltaTMin = deltaTMin;
   }

   public double getGravityX()
   {
      return gravity.getX();
   }

   public void setGravityX(double gravityX)
   {
      this.gravity.setX(gravityX);
   }

   public double getGravityY()
   {
      return gravity.getY();
   }

   public void setGravityY(double gravityY)
   {
      this.gravity.setY(gravityY);
   }

   public double getGravityZ()
   {
      return gravity.getZ();
   }

   public void setGravityZ(double gravityZ)
   {
      this.gravity.setZ(gravityZ);
   }

   public double getRobotMass()
   {
      return robotMass;
   }

   public void setRobotMass(double robotMass)
   {
      this.robotMass = robotMass;
   }

   public double getForceRegulizationWeight()
   {
      return forceRegulizationWeight;
   }

   public void setForceRegularizationWeight(double forceRegulizationWeight)
   {
      this.forceRegulizationWeight = forceRegulizationWeight;
   }

   public double getdForceRegularizationWeight()
   {
      return dForceRegularizationWeight;
   }

   public void setdForceRegularizationWeight(double dForceRegularizationWeight)
   {
      this.dForceRegularizationWeight = dForceRegularizationWeight;
   }

   public void setGravity(Vector3D gravity)
   {
      this.gravity.set(gravity);
   }

   public void getMaxForce(Vector3D maxForceToPack)
   {
      maxForceToPack.set(this.maximumForce);
   }

   public void setMaxForce(Vector3D maxForce)
   {
      this.maximumForce.set(maxForce);
   }

   public void getMinForce(Vector3D minForceToPack)
   {
      minForceToPack.set(this.minimumForce);
   }

   public void setMinForce(Vector3D minForce)
   {
      this.minimumForce.set(minForce);
   }

   public void getMaxForceRate(Vector3D maxForceRateToPack)
   {
      maxForceRateToPack.set(this.maximumForceRate);
   }

   public void setMaxForceRate(Vector3D maxForceRate)
   {
      this.maximumForceRate.set(maxForceRate);
   }

   public void getMinForceRate(Vector3D minForceRateToPack)
   {
      minForceRateToPack.set(this.minimumForceRate);
   }

   public void setMinForceRate(Vector3D minForceRate)
   {
      this.minimumForceRate.set(minForceRate);
   }

   public void getGravity(Vector3D gravityToPack)
   {
      gravityToPack.set(this.gravity);
   }

   public double getOptimizationConvergenceThreshold()
   {
      return 1e-14;
   }

   public double getDefaultMotionPlanningPositionObjecitveWeight()
   {
      return 10.0;
   }

   public double getDefaultMotionPlanningVelocityObjecitveWeight()
   {
      return 10.0;
   }
}
