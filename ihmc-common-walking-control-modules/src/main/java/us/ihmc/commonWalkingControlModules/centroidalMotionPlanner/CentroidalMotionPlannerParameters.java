package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

public class CentroidalMotionPlannerParameters
{
   private double gravityX;
   private double gravityY;
   private double gravityZ;
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
      return gravityX;
   }

   public void setGravityX(double gravityX)
   {
      this.gravityX = gravityX;
   }

   public double getGravityY()
   {
      return gravityY;
   }

   public void setGravityY(double gravityY)
   {
      this.gravityY = gravityY;
   }

   public double getGravityZ()
   {
      return gravityZ;
   }

   public void setGravityZ(double gravityZ)
   {
      this.gravityZ = gravityZ;
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

   public void setForceRegulizationWeight(double forceRegulizationWeight)
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
}
