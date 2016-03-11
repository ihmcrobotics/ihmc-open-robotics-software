package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorFromFastRunner;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FourBarKinematicLoopTools
{   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean DEBUG = false;

   public static void checkJointAxesAreParallelAndSetJointAxis(FrameVector masterAxis, FrameVector jointBAxis, FrameVector jointCAxis, FrameVector jointDAxis)
   {
      masterAxis.changeFrame(worldFrame);
      jointBAxis.changeFrame(worldFrame);
      jointCAxis.changeFrame(worldFrame);
      jointDAxis.changeFrame(worldFrame);
      
      if(DEBUG)
      {
         System.out.println("\nDebugging axis dot products: \nmaster x B = " + masterAxis.dot(jointBAxis) + "\nmaster x C = " + masterAxis.dot(jointCAxis) + "\nmaster x D = " + masterAxis.dot(jointDAxis) );
      }
      
      // Both the exact same axis and a flipped axis are valid (eg: y and -y). So as long as the absolute value of the dot product is 1, the axis are parallel.      
      double epsilon = 1.0e-9;
      boolean isJointBParallel = MathTools.epsilonEquals(Math.abs(masterAxis.dot(jointBAxis)), 1.0, epsilon);
      boolean isJointCParallel = MathTools.epsilonEquals(Math.abs(masterAxis.dot(jointCAxis)), 1.0, epsilon);
      boolean isJointDParallel = MathTools.epsilonEquals(Math.abs(masterAxis.dot(jointDAxis)), 1.0, epsilon);
      
      if (!isJointBParallel || !isJointCParallel || !isJointDParallel)
      {
         throw new RuntimeException("All joints in the four bar must rotate around the same axis!");
      }
   }
   
   public static void verifyMasterJointLimits(String fourBarName, RevoluteJoint masterJointA, FourBarCalculatorFromFastRunner fourBarCalculator)
   {
      double maxValidMasterJointAngle = fourBarCalculator.getMaxDAB();
      double minValidMasterJointAngle = fourBarCalculator.getMinDAB();

      // A) Angle limits not set
      if (masterJointA.getJointLimitLower() == Double.NEGATIVE_INFINITY || masterJointA.getJointLimitUpper() == Double.POSITIVE_INFINITY)
      {
         throw new RuntimeException("Must set the joint limits for the master joint of the " + fourBarName + " four bar.\nNote that for the given link lengths max angle is " + maxValidMasterJointAngle + "and min angle is" + minValidMasterJointAngle);
      }

      // B) Max angle limit is too large

      if (masterJointA.getJointLimitUpper() > maxValidMasterJointAngle)
      {
         throw new RuntimeException("The maximum valid joint angle for the master joint of the " + fourBarName + " four bar is " + maxValidMasterJointAngle + " to avoid flipping, but was set to " + masterJointA.getJointLimitUpper());
      }

      // C) Min angle limit is too small

      if (masterJointA.getJointLimitLower() < minValidMasterJointAngle)
      {
         throw new RuntimeException("The minimum valid joint angle for the master joint of the " + fourBarName + " four bar is " + minValidMasterJointAngle + " to avoid flipping, but was set to " + masterJointA.getJointLimitLower());
      }
   }
}
