package us.ihmc.robotics.screwTheory;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.Axis;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorFromFastRunner;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FourBarKinematicLoop
{
   /*
    * Representation of the four bar with name correspondences.
    * This name convention matches the one used in the FourBarCalculator from fastRunner
    *   
    *              masterL
    *     master=A--------B
    *            |\      /|
    *            | \    / |
    *            |  \  /  |
    *            |   \/   |
    *            |   /\   |
    *            |  /  \  |
    *            | /    \ |
    *            |/      \|
    *            D--------C
    */
   private static final boolean DEBUG = true;

   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final String name;
   private final RevoluteJoint masterJointA;
   private final PassiveRevoluteJoint passiveJointB, passiveJointC, passiveJointD;
   private final Vector3d closurePointFromLastPassiveJointVect;
   private final boolean recomputeJointLimits;
   
   private final FramePoint masterJointAPosition = new FramePoint();
   private final FramePoint jointBPosition = new FramePoint();
   private final FramePoint jointCPosition = new FramePoint();
   private final FramePoint jointDPosition = new FramePoint();
   private double masterLinkAB, BC, CD, DA;
   private final FrameVector vectorAB = new FrameVector();
   private final FrameVector vectorBC = new FrameVector();
   private final FrameVector vectorCD = new FrameVector();
   private final FrameVector vectorDA = new FrameVector();
   private final FrameVector2d vectorBCProjected = new FrameVector2d();
   private final FrameVector2d vectorCDProjected = new FrameVector2d();
   private final FrameVector2d vectorDAProjected = new FrameVector2d();
   private final FrameVector2d vectorABProjected = new FrameVector2d();
   
   private FourBarCalculatorFromFastRunner fourBarCalculator;
   
   private double[] interiorAnglesAtZeroConfiguration = new double[4];
   private double maxValidMasterJointAngle, minValidMasterJointAngle;
   
   private final FrameVector jointAxisInWorld;
   private final FrameVector masterAxis, jointBAxis, jointCAxis, jointDAxis;
   private final ReferenceFrame frameWithZAlongJointAxis;
   
   public FourBarKinematicLoop(String name, RevoluteJoint masterJointA, PassiveRevoluteJoint passiveJointB,
         PassiveRevoluteJoint passiveJointC, PassiveRevoluteJoint passiveJointD, Vector3d closurePointFromLastPassiveJoint, boolean recomputeJointLimits)
   {
      this.name = name;
      this.masterJointA = masterJointA;
      this.passiveJointB = passiveJointB;
      this.passiveJointC = passiveJointC;
      this.passiveJointD = passiveJointD;
      this.closurePointFromLastPassiveJointVect = closurePointFromLastPassiveJoint;
      this.recomputeJointLimits = recomputeJointLimits;
      
      // Rotation axis
      masterAxis = masterJointA.getJointAxis();
      jointBAxis = passiveJointB.getJointAxis();
      jointCAxis = passiveJointC.getJointAxis();
      jointDAxis = passiveJointD.getJointAxis();

      jointAxisInWorld = new FrameVector();
      checkJointAxesAreParallelAndSetJointAxis();
      
      // Joint order
      checkCorrectJointOrder();
      
      // Go to zero configuration
      masterJointA.setQ(0.0);
      passiveJointB.setQ(0.0);
      passiveJointC.setQ(0.0);
      passiveJointD.setQ(0.0);
      
      // Link lengths
      frameWithZAlongJointAxis = ReferenceFrame.constructReferenceFrameFromPointAndAxis(name + "FrameWithZAlongJointAxis", new FramePoint(), Axis.Z, jointAxisInWorld);
      initializeJointPositionsAndLinkVectors();
      masterLinkAB = vectorABProjected.length();
      BC = vectorBCProjected.length();
      CD = vectorCDProjected.length();
      DA = vectorDAProjected.length();
      
      if (DEBUG)
      {
         System.out.println("\nLink length debugging: \n");
         System.out.println("masterLinkAB BC CD DA : " + masterLinkAB + ", " + BC + ", " + CD + ", " + DA);
      }
      
      // Check (and correct, if applicable) joint limits and close the loop
      clipMasterJointLimits(passiveJointB, passiveJointC, passiveJointD);
           
      setInteriorAngleOffsets();

      if(!recomputeJointLimits)
      {
         fourBarCalculator = new FourBarCalculatorFromFastRunner(DA, masterLinkAB, BC, CD, recomputeJointLimits);
      }

      verifyMasterJointLimits();
      
      if (DEBUG)
      {
         System.out.println("\nInitial joint angles debugging:\n\n" + "MasterQ: " + masterJointA.getQ() + "\njointBQ: " + passiveJointB.getQ() + "\njointCQ: "
               + passiveJointC.getQ() + "\njointDQ: " + passiveJointD.getQ() + "\n");
      }
   }

   private void checkJointAxesAreParallelAndSetJointAxis()
   {
      masterAxis.changeFrame(worldFrame);
      jointBAxis.changeFrame(worldFrame);
      jointCAxis.changeFrame(worldFrame);
      jointDAxis.changeFrame(worldFrame);

      // Both the exact same axis and a flipped axis are valid (eg: y and -y). So as long as the absolute value of the dot product is 1, the axis are parallel.      
      if (MathTools.epsilonEquals(Math.abs(masterAxis.dot(jointBAxis)), 1.0, 1.0e-7)
            && MathTools.epsilonEquals(Math.abs(masterAxis.dot(jointCAxis)), 1.0, 1.0e-7)
            && MathTools.epsilonEquals(Math.abs(masterAxis.dot(jointDAxis)), 1.0, 1.0e-7))
      {
         jointAxisInWorld.set(masterAxis);
      }
      else
      {
         throw new RuntimeException("All joints in the four bar must rotate around the same axis!");
      }
   }

   private void checkCorrectJointOrder()
   {
      if (masterJointA.getSuccessor() != passiveJointB.getPredecessor() || passiveJointB.getSuccessor() != passiveJointC.getPredecessor()
            || passiveJointC.getSuccessor() != passiveJointD.getPredecessor())
      {
         throw new RuntimeException("The joints that form the " + name + " four bar must be passed in clockwise or counterclockwise order");
      }

      if (DEBUG)
      {
         System.out.println("\nDebugging  check joint order:\n\nsuccessor \t predecessor\n" + masterJointA.getSuccessor() + "\t  "
               + passiveJointB.getPredecessor() + "\n" + passiveJointB.getSuccessor() + "\t  " + passiveJointC.getPredecessor() + "\n"
               + passiveJointC.getSuccessor() + "\t  " + passiveJointD.getPredecessor() + "\n");
      }
   }

   private void initializeJointPositionsAndLinkVectors()
   {
      jointBPosition.setToZero(passiveJointB.getFrameAfterJoint());
      jointCPosition.setToZero(passiveJointC.getFrameAfterJoint());
      jointDPosition.setToZero(passiveJointD.getFrameAfterJoint());
      masterJointAPosition.setToZero(masterJointA.getFrameAfterJoint());

      jointBPosition.changeFrame(worldFrame);
      jointCPosition.changeFrame(worldFrame);
      jointDPosition.changeFrame(worldFrame);
      masterJointAPosition.changeFrame(worldFrame);

      vectorAB.sub(jointBPosition, masterJointAPosition);
      vectorBC.sub(jointCPosition, jointBPosition);
      vectorCD.sub(jointDPosition, jointCPosition);
      vectorDA.setIncludingFrame(passiveJointD.getFrameAfterJoint(), closurePointFromLastPassiveJointVect);
      
      vectorAB.changeFrame(frameWithZAlongJointAxis);
      vectorBC.changeFrame(frameWithZAlongJointAxis);
      vectorCD.changeFrame(frameWithZAlongJointAxis);
      vectorDA.changeFrame(frameWithZAlongJointAxis);
      
      vectorBCProjected.set(vectorBC.getX(), vectorBC.getY());
      vectorCDProjected.set(vectorCD.getX(), vectorCD.getY());
      vectorDAProjected.set(vectorDA.getX(), vectorDA.getY());
      vectorABProjected.set(vectorAB.getX(), vectorAB.getY());
      
      if (DEBUG)
      {  
      System.out.println("\nJoint to joint vectors debugging:\n");
      System.out.println("vector ab = " + vectorAB);
      System.out.println("vector bc = " + vectorBC);
      System.out.println("vector cd = " + vectorCD);
      System.out.println("vector da = " + vectorDA);
      }
   }
   
   private void setInteriorAngleOffsets()
   {
      vectorDAProjected.changeFrame(worldFrame);
      vectorABProjected.changeFrame(worldFrame);
      vectorBCProjected.changeFrame(worldFrame);
      vectorCDProjected.changeFrame(worldFrame);

      interiorAnglesAtZeroConfiguration[1] = Math.PI - vectorABProjected.angle(vectorBCProjected);
      interiorAnglesAtZeroConfiguration[2] = Math.PI - vectorBCProjected.angle(vectorCDProjected);
      interiorAnglesAtZeroConfiguration[3] = Math.PI - vectorCDProjected.angle(vectorDAProjected);
           
      if (DEBUG)
      {  
         System.out.println("\nOffset angle debugging:\n");
         System.out.println("offset A = " + interiorAnglesAtZeroConfiguration[0]);
         System.out.println("offset B = " + interiorAnglesAtZeroConfiguration[1]);
         System.out.println("offset C = " + interiorAnglesAtZeroConfiguration[2]);
         System.out.println("offset D = " + interiorAnglesAtZeroConfiguration[3]);
      }
   }

   //TODO avoid repetition
   
   /**
    * Clips the range of motion of the master joint if the limits set for the joint that is passed in are more restrictive
    */
   private void clipMasterJointLimits(PassiveRevoluteJoint jointB, PassiveRevoluteJoint jointC, PassiveRevoluteJoint jointD)
   {

      if (jointB.getJointLimitLower() != Double.NEGATIVE_INFINITY || jointB.getJointLimitUpper() != Double.POSITIVE_INFINITY)
      {
         fourBarCalculator.computeMasterJointAngleGivenAngleABC(jointB.getJointLimitLower());

         if (fourBarCalculator.getAngleDAB() > minValidMasterJointAngle)
            minValidMasterJointAngle = fourBarCalculator.getAngleDAB();

         fourBarCalculator.computeMasterJointAngleGivenAngleABC(jointB.getJointLimitUpper());

         if (fourBarCalculator.getAngleDAB() < maxValidMasterJointAngle)
            maxValidMasterJointAngle = fourBarCalculator.getAngleDAB();
      }
      
      if (jointC.getJointLimitLower() != Double.NEGATIVE_INFINITY || jointC.getJointLimitUpper() != Double.POSITIVE_INFINITY)
      {
         fourBarCalculator.computeMasterJointAngleGivenAngleBCD(jointC.getJointLimitLower());

         if (fourBarCalculator.getAngleDAB() > minValidMasterJointAngle)
            minValidMasterJointAngle = fourBarCalculator.getAngleDAB();

         fourBarCalculator.computeMasterJointAngleGivenAngleBCD(jointC.getJointLimitUpper());

         if (fourBarCalculator.getAngleDAB() < maxValidMasterJointAngle)
            maxValidMasterJointAngle = fourBarCalculator.getAngleDAB();
      }
      
      if (jointD.getJointLimitLower() != Double.NEGATIVE_INFINITY || jointD.getJointLimitUpper() != Double.POSITIVE_INFINITY)
      {
         fourBarCalculator.computeMasterJointAngleGivenAngleCDA(jointD.getJointLimitLower());

         if (fourBarCalculator.getAngleDAB() > minValidMasterJointAngle)
            minValidMasterJointAngle = fourBarCalculator.getAngleDAB();

         fourBarCalculator.computeMasterJointAngleGivenAngleCDA(jointD.getJointLimitUpper());

         if (fourBarCalculator.getAngleDAB() < maxValidMasterJointAngle)
            maxValidMasterJointAngle = fourBarCalculator.getAngleDAB();
      }

   }
     
   
   private void verifyMasterJointLimits() 
   {
      maxValidMasterJointAngle = fourBarCalculator.getMinDAB();
      minValidMasterJointAngle = fourBarCalculator.getMaxDAB();

      if (DEBUG)
      {
         System.out.println("\nMax master joint angle: " + maxValidMasterJointAngle);
         System.out.println("Min master joint angle: " + minValidMasterJointAngle);
      }

      // 1 - Angle limits not set
      if (masterJointA.getJointLimitLower() == Double.NEGATIVE_INFINITY || masterJointA.getJointLimitUpper() == Double.POSITIVE_INFINITY)
      {
         if (recomputeJointLimits)
         {
            masterJointA.setJointLimitLower(minValidMasterJointAngle);
            masterJointA.setJointLimitUpper(maxValidMasterJointAngle);
            System.out.println("NOTE: The master joint limits have been set to " + minValidMasterJointAngle + " and " + maxValidMasterJointAngle);
         }
         else
         {
            throw new RuntimeException("Must set the joint limits for the master joint of the " + name
                  + " four bar.\nNote that for the given link lengths max angle is " + maxValidMasterJointAngle + "and min angle is" + minValidMasterJointAngle);        
         }
      }

      // 2 - Max angle limit is too large
      if (BC + CD > masterLinkAB + DA)
      {
         if (masterJointA.getJointLimitUpper() > maxValidMasterJointAngle && !recomputeJointLimits)
         {
            throw new RuntimeException("The maximum valid joint angle for the master joint of the " + name + " four bar is " + maxValidMasterJointAngle
                  + " to avoid flipping, but was set to " + masterJointA.getJointLimitUpper());
         }
      }
      else if (masterJointA.getJointLimitUpper() > Math.PI && !recomputeJointLimits)
      {
         throw new RuntimeException("The maximum valid joint angle for the master joint of the " + name + " four bar is " + maxValidMasterJointAngle
               + " to avoid flipping, but was set to " + masterJointA.getJointLimitUpper());
      }

      // 3 - Min angle limit is too small
      if (masterLinkAB + BC < DA + CD)
      {
         if (masterJointA.getJointLimitLower() < minValidMasterJointAngle && !recomputeJointLimits)
         {
            throw new RuntimeException("The minimum valid joint angle for the master joint of the " + name + " four bar is " + minValidMasterJointAngle
                  + " to avoid flipping, but was set to " + masterJointA.getJointLimitLower());
         }
      }
      else if (masterJointA.getJointLimitLower() < 0.0 && !recomputeJointLimits)
      {
         throw new RuntimeException("The minimum valid joint angle for the master joint of the " + name + " four bar is " + minValidMasterJointAngle
               + " to avoid flipping, but was set to " + masterJointA.getJointLimitLower());
      }
   }

   public void updateAnglesAndVelocities()
   {
      fourBarCalculator.updateAnglesAndVelocitiesGivenAngleDAB(masterJointA.getQ(), masterJointA.getQd());
      passiveJointB.setQ(fourBarCalculator.getAngleABC() - interiorAnglesAtZeroConfiguration[1]);
      passiveJointC.setQ(fourBarCalculator.getAngleBCD() - interiorAnglesAtZeroConfiguration[2]);
      passiveJointD.setQ(fourBarCalculator.getAngleCDA() - interiorAnglesAtZeroConfiguration[3]);
      passiveJointB.setQd(fourBarCalculator.getAngleDtABC());
      passiveJointC.setQd(fourBarCalculator.getAngleDtBCD());
      passiveJointD.setQd(fourBarCalculator.getAngleDtCDA());
   }
}
