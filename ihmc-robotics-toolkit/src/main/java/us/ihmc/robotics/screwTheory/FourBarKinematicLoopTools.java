package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.commons.MathTools;

public class FourBarKinematicLoopTools
{   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean DEBUG = false;

   public static void checkJointAxesAreParallel(FrameVector3D masterAxis, FrameVector3D jointBAxis, FrameVector3D jointCAxis, FrameVector3D jointDAxis)
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

   public static PassiveRevoluteJoint getFourBarOutputJoint(PassiveRevoluteJoint passiveJointB, PassiveRevoluteJoint passiveJointC, PassiveRevoluteJoint passiveJointD)
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
   
   public static boolean checkFourBarConvexityAndOrientation(FrameVector2D vectorABProjected, FrameVector2D vectorBCProjected, FrameVector2D vectorCDProjected, FrameVector2D vectorDAProjected)
   {
      boolean ccwConvex = vectorABProjected.cross(vectorBCProjected) > 0.0 && vectorBCProjected.cross(vectorCDProjected) > 0.0 && vectorCDProjected.cross(vectorDAProjected) > 0.0;
      boolean cwConvex = vectorABProjected.cross(vectorBCProjected) < 0.0 && vectorBCProjected.cross(vectorCDProjected) < 0.0 && vectorCDProjected.cross(vectorDAProjected) < 0.0;

      if (!ccwConvex && !cwConvex)
      {
         throw new RuntimeException("At q = 0 the fourBar must be convex");
      }  
      
      return cwConvex;
   }
}
