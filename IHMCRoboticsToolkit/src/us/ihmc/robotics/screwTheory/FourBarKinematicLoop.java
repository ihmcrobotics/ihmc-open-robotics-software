package us.ihmc.robotics.screwTheory;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.Axis;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.AngleTools;
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
   private static final boolean DEBUG = false;

   private final RevoluteJoint masterJointA;
   private final PassiveRevoluteJoint passiveJointB, passiveJointC, passiveJointD;
   private final Vector3d closurePointFromLastPassiveJointVect;  
   
   private final FourBarCalculatorFromFastRunner fourBarCalculator;
   
   private final double[] interiorAnglesAtZeroConfiguration;
   
   private final ReferenceFrame frameBeforeFourBarWithZAlongJointAxis;
   
   public FourBarKinematicLoop(String name, RevoluteJoint masterJointA, PassiveRevoluteJoint passiveJointB,
         PassiveRevoluteJoint passiveJointC, PassiveRevoluteJoint passiveJointD, Vector3d closurePointFromLastPassiveJoint, boolean recomputeJointLimits)
   {
      this.masterJointA = masterJointA;
      this.passiveJointB = passiveJointB;
      this.passiveJointC = passiveJointC;
      this.passiveJointD = passiveJointD;
      this.closurePointFromLastPassiveJointVect = new Vector3d(closurePointFromLastPassiveJoint);
      
      // Rotation axis
      FrameVector masterJointAxis = masterJointA.getJointAxis();
      masterJointAxis.changeFrame(masterJointA.getFrameBeforeJoint());
      frameBeforeFourBarWithZAlongJointAxis = ReferenceFrame.constructReferenceFrameFromPointAndAxis(name + "FrameWithZAlongJointAxis", new FramePoint(masterJointA.getFrameBeforeJoint()), Axis.Z, masterJointAxis);

      FrameVector masterAxis = masterJointA.getJointAxis();
      FrameVector jointBAxis = passiveJointB.getJointAxis();
      FrameVector jointCAxis = passiveJointC.getJointAxis();
      FrameVector jointDAxis = passiveJointD.getJointAxis();
      FourBarKinematicLoopTools.checkJointAxesAreParallelAndSetJointAxis(masterAxis, jointBAxis, jointCAxis, jointDAxis);
      
      // Joint order
      FourBarKinematicLoopTools.checkCorrectJointOrder(name, masterJointA, passiveJointB, passiveJointC, passiveJointD);
      
      // Go to zero configuration
      masterJointA.setQ(0.0);
      passiveJointB.setQ(0.0);
      passiveJointC.setQ(0.0);
      passiveJointD.setQ(0.0);

      // Link lengths
      FrameVector2d vectorBCProjected = new FrameVector2d();
      FrameVector2d vectorCDProjected = new FrameVector2d();
      FrameVector2d vectorDAProjected = new FrameVector2d();
      FrameVector2d vectorABProjected = new FrameVector2d();
      computeJointToJointVectorProjectedOntoFourBarPlane(vectorBCProjected, vectorCDProjected, vectorDAProjected, vectorABProjected);

      // Calculator
      fourBarCalculator = createFourBarCalculator(vectorBCProjected, vectorCDProjected, vectorDAProjected, vectorABProjected);

      // Initialize interior angles
      interiorAnglesAtZeroConfiguration = computeInteriorAnglesAtZeroConfiguration(vectorDAProjected, vectorABProjected, vectorBCProjected, vectorCDProjected);

      // Find the most conservative limits for the master joint angle (A) and reset them if necessary
      if (recomputeJointLimits) 
      {
         /* - If the limits for B, C, and/or D are given and are more restrictive than those of A crop the value of Amax and/or Amin. 
          * - Else if the limits given for A are the most restrictive of all, keep them.
          * - Else set the limits to the value given by the calculator. 
          */
         double minValidMasterJointAngle = computeMinValidMasterJointAngle(masterJointA, passiveJointB, passiveJointC, passiveJointD);
         double maxValidMasterJointAngle = computeMaxValidMasterJointAngle(masterJointA, passiveJointB, passiveJointC, passiveJointD);         

         masterJointA.setJointLimitLower(minValidMasterJointAngle);
         masterJointA.setJointLimitUpper(maxValidMasterJointAngle);
         
         if(DEBUG)
         {
            System.out.println("NOTE: The master joint limits have been set to " + minValidMasterJointAngle + " and " + maxValidMasterJointAngle);            
         }
      }
      else
      {
         FourBarKinematicLoopTools.verifyMasterJointLimits(name, masterJointA, fourBarCalculator);

         if (DEBUG)
         {
            System.out.println("\nMax master joint angle: " + masterJointA.getJointLimitUpper());
            System.out.println("Min master joint angle: " + masterJointA.getJointLimitLower());
         }
      }
      
      if (DEBUG)
      {
         double qA = masterJointA.getQ();
         double qB = passiveJointB.getQ();
         double qC = passiveJointC.getQ();
         double qD = passiveJointD.getQ();
         System.out.println("\nInitial joint angles debugging:\n\n" + "MasterQ: " + qA + "\njointBQ: " + qB + "\njointCQ: " + qC + "\njointDQ: " + qD + "\n");
      }
   }

   private void computeJointToJointVectorProjectedOntoFourBarPlane(FrameVector2d vectorBCProjectedToPack, FrameVector2d vectorCDProjectedToPack, FrameVector2d vectorDAProjectedToPack, FrameVector2d vectorABProjectedToPack)
   {
      FramePoint masterJointAPosition = new FramePoint(masterJointA.getFrameAfterJoint());
      FramePoint jointBPosition = new FramePoint(passiveJointB.getFrameAfterJoint());
      FramePoint jointCPosition = new FramePoint(passiveJointC.getFrameAfterJoint());
      FramePoint jointDPosition = new FramePoint(passiveJointD.getFrameAfterJoint());

      jointBPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointCPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointDPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      masterJointAPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);

      FrameVector vectorAB = new FrameVector(frameBeforeFourBarWithZAlongJointAxis);
      FrameVector vectorBC = new FrameVector(frameBeforeFourBarWithZAlongJointAxis);
      FrameVector vectorCD = new FrameVector(frameBeforeFourBarWithZAlongJointAxis);
      FrameVector vectorDA = new FrameVector(frameBeforeFourBarWithZAlongJointAxis);

      vectorAB.sub(jointBPosition, masterJointAPosition);
      vectorBC.sub(jointCPosition, jointBPosition);
      vectorCD.sub(jointDPosition, jointCPosition);
      vectorDA.setIncludingFrame(passiveJointD.getFrameAfterJoint(), closurePointFromLastPassiveJointVect);
      vectorDA.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      
      vectorBCProjectedToPack.setByProjectionOntoXYPlaneIncludingFrame(vectorBC);
      vectorCDProjectedToPack.setByProjectionOntoXYPlaneIncludingFrame(vectorCD);
      vectorDAProjectedToPack.setByProjectionOntoXYPlaneIncludingFrame(vectorDA);
      vectorABProjectedToPack.setByProjectionOntoXYPlaneIncludingFrame(vectorAB);
      
      if (DEBUG)
      {
         System.out.println("\nJoint to joint vectors debugging:\n");
         System.out.println("vector ab = " + vectorAB);
         System.out.println("vector bc = " + vectorBC);
         System.out.println("vector cd = " + vectorCD);
         System.out.println("vector da = " + vectorDA);
      }
   }

   private FourBarCalculatorFromFastRunner createFourBarCalculator(FrameVector2d vectorBCProjected, FrameVector2d vectorCDProjected, FrameVector2d vectorDAProjected, FrameVector2d vectorABProjected)
   {
      double masterLinkAB = vectorABProjected.length();
      double BC = vectorBCProjected.length();
      double CD = vectorCDProjected.length();
      double DA = vectorDAProjected.length();
      
      if (DEBUG)
      {
         System.out.println("\nLink length debugging: \n");
         System.out.println("masterLinkAB BC CD DA : " + masterLinkAB + ", " + BC + ", " + CD + ", " + DA);
      }

      FourBarCalculatorFromFastRunner fourBarCalculator = new FourBarCalculatorFromFastRunner(DA, masterLinkAB, BC, CD);

      return fourBarCalculator;
   }

   private double[] computeInteriorAnglesAtZeroConfiguration(FrameVector2d vectorDAProjected, FrameVector2d vectorABProjected, FrameVector2d vectorBCProjected, FrameVector2d vectorCDProjected)
   {
      vectorDAProjected.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      vectorABProjected.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      vectorBCProjected.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      vectorCDProjected.changeFrame(frameBeforeFourBarWithZAlongJointAxis);

      FrameVector jointBAxis = passiveJointB.getJointAxis();
      FrameVector jointCAxis = passiveJointC.getJointAxis();
      FrameVector jointDAxis = passiveJointD.getJointAxis();

      jointBAxis.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointCAxis.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointDAxis.changeFrame(frameBeforeFourBarWithZAlongJointAxis); 
      
      double jointBAxisZ = jointBAxis.getZ();
      double jointCAxisZ = jointCAxis.getZ();
      double jointDAxisZ = jointDAxis.getZ();
      
      Vector2d tempVectorAB = new Vector2d();
      Vector2d tempVectorBC = new Vector2d();
      Vector2d tempVectorCD = new Vector2d();
      Vector2d tempVectorDA = new Vector2d();
      
      vectorABProjected.get(tempVectorAB);
      vectorBCProjected.get(tempVectorBC);
      vectorCDProjected.get(tempVectorCD);
      vectorDAProjected.get(tempVectorDA);

      double[] interiorAnglesAtZeroConfiguration = new double[3];
      interiorAnglesAtZeroConfiguration[0] = Math.PI + jointBAxisZ * AngleTools.angleMinusPiToPi(tempVectorAB, tempVectorBC);
      interiorAnglesAtZeroConfiguration[1] = Math.PI + jointCAxisZ * AngleTools.angleMinusPiToPi(tempVectorBC, tempVectorCD);
      interiorAnglesAtZeroConfiguration[2] = Math.PI + jointDAxisZ * AngleTools.angleMinusPiToPi(tempVectorCD, tempVectorDA);
            
      if (DEBUG)
      {  
         System.out.println("\nOffset angle debugging:\n");
         System.out.println("offset B = " + interiorAnglesAtZeroConfiguration[0]);
         System.out.println("offset C = " + interiorAnglesAtZeroConfiguration[1]);
         System.out.println("offset D = " + interiorAnglesAtZeroConfiguration[2]);
      }

      return interiorAnglesAtZeroConfiguration;
   }
   
   /**
    * Clips the min master joint angle if the lower limit for any of the joints that are passed in is more restrictive
    */
   private double computeMinValidMasterJointAngle(RevoluteJoint masterJointA, PassiveRevoluteJoint jointB, PassiveRevoluteJoint jointC, PassiveRevoluteJoint jointD)
   {
      double minValidMasterJointAngle = fourBarCalculator.getMinDAB();

      if (masterJointA.getJointLimitLower() != Double.NEGATIVE_INFINITY)
      {
         double minAngleASetByUser = masterJointA.getJointLimitLower();

         if (MathTools.isInsideBoundsExclusive(minAngleASetByUser, 0.0, Math.PI))
         {
            minValidMasterJointAngle = Math.max(minAngleASetByUser, minValidMasterJointAngle);
         }
      }
      
      if (jointB.getJointLimitUpper() != Double.POSITIVE_INFINITY)
      {
         double maxAngleB = jointB.getJointLimitUpper() + interiorAnglesAtZeroConfiguration[0];

         if (MathTools.isInsideBoundsExclusive(maxAngleB, 0.0, Math.PI))
         {
            fourBarCalculator.computeMasterJointAngleGivenAngleABC(maxAngleB);
            minValidMasterJointAngle = Math.max(minValidMasterJointAngle, fourBarCalculator.getAngleDAB());
         }
      }

      if (jointC.getJointLimitLower() != Double.NEGATIVE_INFINITY)
      {
         double minAngleC = jointC.getJointLimitLower() + interiorAnglesAtZeroConfiguration[1];
         
         if (MathTools.isInsideBoundsExclusive(minAngleC, 0.0, Math.PI))
         {
            fourBarCalculator.computeMasterJointAngleGivenAngleBCD(minAngleC);
            minValidMasterJointAngle = Math.max(minValidMasterJointAngle, fourBarCalculator.getAngleDAB());
         }
      }

      if (jointD.getJointLimitUpper() != Double.POSITIVE_INFINITY)
      {
         double maxAngleD = jointD.getJointLimitUpper() + interiorAnglesAtZeroConfiguration[2];

         if (MathTools.isInsideBoundsExclusive(maxAngleD, 0.0, Math.PI))
         {
            fourBarCalculator.computeMasterJointAngleGivenAngleCDA(maxAngleD);
            minValidMasterJointAngle = Math.max(minValidMasterJointAngle, fourBarCalculator.getAngleDAB());
         }
      }

      return minValidMasterJointAngle;
   }
   
   /**
    * Clips the max master joint angle if the upper limit for any of the joints that are passed in is more restrictive
    */
   private double computeMaxValidMasterJointAngle(RevoluteJoint masterJointA, PassiveRevoluteJoint jointB, PassiveRevoluteJoint jointC, PassiveRevoluteJoint jointD)
   {
      double maxValidMasterJointAngle = fourBarCalculator.getMaxDAB();

      if (masterJointA.getJointLimitUpper() != Double.POSITIVE_INFINITY)
      {
         double maxAngleASetByUser = masterJointA.getJointLimitUpper();

         if (MathTools.isInsideBoundsExclusive(maxAngleASetByUser, 0.0, Math.PI))
         {
            maxValidMasterJointAngle = Math.min(maxAngleASetByUser, maxValidMasterJointAngle);
         }
      }
      
      if (jointB.getJointLimitLower() != Double.NEGATIVE_INFINITY)
      {
         double minAngleB = jointB.getJointLimitLower() + interiorAnglesAtZeroConfiguration[0];

         if (MathTools.isInsideBoundsExclusive(minAngleB, 0.0, Math.PI))
         {
            fourBarCalculator.computeMasterJointAngleGivenAngleABC(minAngleB);
            maxValidMasterJointAngle = Math.min(maxValidMasterJointAngle, fourBarCalculator.getAngleDAB());
         }
      }

      if (jointC.getJointLimitUpper() != Double.POSITIVE_INFINITY)
      {
         double maxAngleC = jointC.getJointLimitUpper() + interiorAnglesAtZeroConfiguration[1];
         
         if (MathTools.isInsideBoundsExclusive(maxAngleC, 0.0, Math.PI))
         {
            fourBarCalculator.computeMasterJointAngleGivenAngleBCD(maxAngleC);
            maxValidMasterJointAngle = Math.min(maxValidMasterJointAngle, fourBarCalculator.getAngleDAB());
         }
      }

      if (jointD.getJointLimitLower() != Double.NEGATIVE_INFINITY)
      {
         double minAngleD = jointD.getJointLimitLower() + interiorAnglesAtZeroConfiguration[2];
         
         if (MathTools.isInsideBoundsExclusive(minAngleD, 0.0, Math.PI))
         {
            fourBarCalculator.computeMasterJointAngleGivenAngleCDA(minAngleD);
            maxValidMasterJointAngle = Math.min(maxValidMasterJointAngle, fourBarCalculator.getAngleDAB());
         }
      }

      return maxValidMasterJointAngle;
   }
   
   public void updateAnglesAndVelocities()
   {
      double currentMasterJointA = masterJointA.getQ();
      if (currentMasterJointA < masterJointA.getJointLimitLower() || currentMasterJointA > masterJointA.getJointLimitUpper())
      {
         throw new RuntimeException(
               masterJointA.getName() + " is set outside of its bounds [" + masterJointA.getJointLimitLower() + ", " + masterJointA.getJointLimitUpper() + "]");
      }
      
      fourBarCalculator.updateAnglesAndVelocitiesGivenAngleDAB(masterJointA.getQ(), masterJointA.getQd());
      passiveJointB.setQ(fourBarCalculator.getAngleABC() - interiorAnglesAtZeroConfiguration[0]);
      passiveJointC.setQ(fourBarCalculator.getAngleBCD() - interiorAnglesAtZeroConfiguration[1]);
      passiveJointD.setQ(fourBarCalculator.getAngleCDA() - interiorAnglesAtZeroConfiguration[2]);
      passiveJointB.setQd(fourBarCalculator.getAngleDtABC());
      passiveJointC.setQd(fourBarCalculator.getAngleDtBCD());
      passiveJointD.setQd(fourBarCalculator.getAngleDtCDA());
   }
   
   public PassiveRevoluteJoint getPassiveRevoluteJointB()
   {
      return passiveJointB;
   }
   
   public PassiveRevoluteJoint getPassiveRevoluteJointC()
   {
      return passiveJointC;
   }
   
   public PassiveRevoluteJoint getPassiveRevoluteJointD()
   {
      return passiveJointD;
   }
}
