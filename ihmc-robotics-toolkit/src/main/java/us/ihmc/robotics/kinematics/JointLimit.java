package us.ihmc.robotics.kinematics;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointLimit 
{
   private double upperPositionLimit;
   private double lowerPositionLimit;
   private double rangeOfMotion;
   
   private double softUpperPositionLimit;
   private double softLowerPositionLimit;
   private double softLimitPercentageOfFullRangeOfMotion;
   private double softLimitRangeOfMotion;
   
   private double velocityLimit;
   private double torqueLimit;
   
   public JointLimit(double upperPositionLimit, double lowerPositionLimit)
   {
      this.upperPositionLimit = upperPositionLimit;     
      this.lowerPositionLimit = lowerPositionLimit;     
                              
      softUpperPositionLimit = upperPositionLimit; 
      softLowerPositionLimit = lowerPositionLimit; 
                              
      velocityLimit = Double.POSITIVE_INFINITY;          
      torqueLimit = Double.POSITIVE_INFINITY;   
   }
   
   public JointLimit(OneDoFJoint oneDoFJoint)
   {
      upperPositionLimit = oneDoFJoint.getJointLimitUpper();     
      lowerPositionLimit = oneDoFJoint.getJointLimitLower(); 
      rangeOfMotion = upperPositionLimit - lowerPositionLimit;
      
      softLimitPercentageOfFullRangeOfMotion = 1.0;
      softUpperPositionLimit = upperPositionLimit; 
      softLowerPositionLimit = lowerPositionLimit; 
      softLimitRangeOfMotion = softUpperPositionLimit - softLowerPositionLimit;
      
      velocityLimit = Double.POSITIVE_INFINITY;          
      torqueLimit = Double.POSITIVE_INFINITY;   
   }


   public double getUpperPositionLimit()
   {
      return upperPositionLimit;
   }


   public void setUpperPositionLimit(double upperPositionLimit)
   {
      this.upperPositionLimit = upperPositionLimit;
   }


   public double getLowerPositionLimit()
   {
      return lowerPositionLimit;
   }


   public void setLowerPositionLimit(double lowerPositionLimit)
   {
      this.lowerPositionLimit = lowerPositionLimit;
   }


   public double getSoftUpperPositionLimit()
   {
      return softUpperPositionLimit;
   }


   public void setSoftUpperPositionLimit(double softUpperPositionLimit)
   {
      this.softUpperPositionLimit = softUpperPositionLimit;
   }


   public double getSoftLowerPositionLimit()
   {
      return softLowerPositionLimit;
   }


   public void setSoftLowerPositionLimit(double softLowerPositionLimit)
   {
      this.softLowerPositionLimit = softLowerPositionLimit;
   }


   public double getVelocityLimit()
   {
      return velocityLimit;
   }


   public void setVelocityLimit(double velocityLimit)
   {
      this.velocityLimit = velocityLimit;
   }


   public double getTorqueLimit()
   {
      return torqueLimit;
   }


   public void setTorqueLimit(double torqueLimit)
   {
      this.torqueLimit = torqueLimit;
   }

   public double getRangeOfMotion()
   {
      return rangeOfMotion;
   }

   public void setRangeOfMotion(double rangeOfMotion)
   {
      this.rangeOfMotion = rangeOfMotion;
   }

   public double getSoftLimitPercentageOfFullRangeOfMotion()
   {
      return softLimitPercentageOfFullRangeOfMotion;
   }

   public void setSoftLimitPercentageOfFullRangeOfMotion(double softLimitPercentageOfFullRangeOfMotion)
   {
      this.softLimitPercentageOfFullRangeOfMotion = softLimitPercentageOfFullRangeOfMotion;
   }

   public double getSoftLimitRangeOfMotion()
   {
      return softLimitRangeOfMotion;
   }

   public void setSoftLimitRangeOfMotion(double softLimitRangeOfMotion)
   {
      this.softLimitRangeOfMotion = softLimitRangeOfMotion;
   }
}
