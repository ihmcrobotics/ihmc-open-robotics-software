package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorWithDerivatives;
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
   
   public static void checkCorrectJointOrder(String fourBarName, RevoluteJoint masterJointA, PassiveRevoluteJoint passiveJointB, PassiveRevoluteJoint passiveJointC, PassiveRevoluteJoint passiveJointD)
   {
      boolean successorAisPredecessorB = masterJointA.getSuccessor() == passiveJointB.getPredecessor();
      boolean successorBisPredecessorC = passiveJointB.getSuccessor() == passiveJointC.getPredecessor();
      boolean succesorCisPredecessorD = passiveJointC.getSuccessor() == passiveJointD.getPredecessor();
      
      if (!successorAisPredecessorB || !successorBisPredecessorC || !succesorCisPredecessorD)
      {
         throw new RuntimeException("The joints that form the " + fourBarName + " four bar must be passed in clockwise or counterclockwise order");
      }

      if (DEBUG)
      {
         System.out.println("\nDebugging  check joint order:\n\nsuccessor \t predecessor\n" + masterJointA.getSuccessor() + "\t  "
               + passiveJointB.getPredecessor() + "\n" + passiveJointB.getSuccessor() + "\t  " + passiveJointC.getPredecessor() + "\n"
               + passiveJointC.getSuccessor() + "\t  " + passiveJointD.getPredecessor() + "\n");
      }
   }
   
   public static void verifyMasterJointLimits(String fourBarName, RevoluteJoint masterJointA, FourBarCalculatorWithDerivatives fourBarCalculator)
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
   
   public static PassiveRevoluteJoint setFourBarOutputJoint(PassiveRevoluteJoint passiveJointB, PassiveRevoluteJoint passiveJointC, PassiveRevoluteJoint passiveJointD)
   {
      // If the output joint is D then it will have at least 1 child, otherwise it won't have any
      if(passiveJointD.getSuccessor().hasChildrenJoints())
      {
         return passiveJointD;
      }
      // Joint C wil only have joint D as its child, unless it's the output joint of the fourbar
      else if (passiveJointC.getSuccessor().getChildrenJoints().size() > 1)
      {
         return passiveJointC;
      }
      else
      {
         return passiveJointB;
      }     
   }
}
