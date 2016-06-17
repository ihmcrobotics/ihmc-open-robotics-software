package us.ihmc.robotics.screwTheory;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.Axis;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorWithDerivatives;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FourBarKinematicLoop
{
   /*
    * Representation of the four bar with name correspondences. This name
    * convention matches the one used in the FourBarCalculatorWithDerivatives
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
    *            
    * Note: zero configuration corresponds to vertical links. Positive angles clockwise.             
    */
   
   private static final boolean DEBUG = true;
   private static final boolean DEFAULT_OUTPUT_jOINT = true; //temporary variable to test loop while it's not part of a larger kinematic chain

   private final RevoluteJoint masterJointA;
   private final PassiveRevoluteJoint passiveJointB, passiveJointC, passiveJointD, fourBarOutputJoint;
   private final Vector3d jointDInJointABeforeFrame;

   private final FourBarCalculatorWithDerivatives fourBarCalculator;

   private final FourBarKinematicLoopJacobianSolver fourBarJacobianSolver;
   private DenseMatrix64F jacobian, outputJointVelocitiesToPack;

   private final double[] interiorAnglesAtZeroConfiguration = new double[4];
   private final double[] passiveJointSigns = new double[4];

   private final ReferenceFrame frameBeforeFourBarWithZAlongJointAxis;
   
   private final boolean fourBarClockwise;

   /**
    * <dt><b>Precondition:</b><dd>
    * The four joints must form a convex quadrilateral at q=0
    * */
   public FourBarKinematicLoop(String name, RevoluteJoint masterJointA, PassiveRevoluteJoint passiveJointB, PassiveRevoluteJoint passiveJointC, PassiveRevoluteJoint passiveJointD, Vector3d jointDInJointABeforeFrame,
         boolean recomputeJointLimits)
   {
      this.masterJointA = masterJointA;
      this.passiveJointB = passiveJointB;
      this.passiveJointC = passiveJointC;
      this.passiveJointD = passiveJointD;
      this.jointDInJointABeforeFrame = new Vector3d(jointDInJointABeforeFrame);
      
      // Rotation axis
      FrameVector masterJointAxis = masterJointA.getJointAxis();
      masterJointAxis.changeFrame(masterJointA.getFrameBeforeJoint());
      frameBeforeFourBarWithZAlongJointAxis = ReferenceFrame.constructReferenceFrameFromPointAndAxis(name + "FrameWithZAlongJointAxis", new FramePoint(masterJointA.getFrameBeforeJoint()), Axis.Z, masterJointAxis);
   
      FrameVector masterAxis = masterJointA.getJointAxis();
      FrameVector jointBAxis = passiveJointB.getJointAxis();
      FrameVector jointCAxis = passiveJointC.getJointAxis();
      FrameVector jointDAxis = passiveJointD.getJointAxis();
      FourBarKinematicLoopTools.checkJointAxesAreParallel(masterAxis, jointBAxis, jointCAxis, jointDAxis);

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
      
      // Check four bar orientation
      fourBarClockwise = isFourBarClockwise(vectorDAProjected, vectorABProjected);

      System.out.println("four bar clockwise is " + fourBarClockwise);

      // Calculator
      fourBarCalculator = createFourBarCalculator(vectorBCProjected, vectorCDProjected, vectorDAProjected, vectorABProjected);

      // Set output joint     
      if(DEFAULT_OUTPUT_jOINT)
      {
         fourBarOutputJoint = passiveJointC;
         System.out.println("\nNote! Output joint set to default: " + fourBarOutputJoint.name + "\n");
      }
      else
      {
         fourBarOutputJoint = FourBarKinematicLoopTools.setFourBarOutputJoint(passiveJointB, passiveJointC, passiveJointD);
      }   

      // Jacobian        
      fourBarJacobianSolver = new FourBarKinematicLoopJacobianSolver(fourBarCalculator, getJointsForJacobianCalculation(fourBarOutputJoint), fourBarOutputJoint);
      jacobian = fourBarJacobianSolver.computeJacobian(fourBarOutputJoint);      
      
      // Initialize interior angle offsets and signs
      initializeInteriorAnglesAtZeroConfigurationAndJointSigns(vectorDAProjected, vectorABProjected, vectorBCProjected, vectorCDProjected);     
      
      // Find the most conservative limits for the master joint angle (A) and reset them if necessary
      if (recomputeJointLimits)
      {
         /*
          * - If the limits for B, C, and/or D are given and are more restrictive than those of A crop the value of Amax and/or Amin. 
          * - Else if the limits given for A are the most restrictive of all, keep them. 
          * - Else set the limits to the value given by the calculator.
          */
         double minValidMasterJointAngle = computeMinValidMasterJointAngle(masterJointA, passiveJointB, passiveJointC, passiveJointD);
         double maxValidMasterJointAngle = computeMaxValidMasterJointAngle(masterJointA, passiveJointB, passiveJointC, passiveJointD);

         masterJointA.setJointLimitLower(minValidMasterJointAngle);
         masterJointA.setJointLimitUpper(maxValidMasterJointAngle);

         if (DEBUG)
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

   private static boolean isFourBarClockwise(FrameVector2d frameVectorDA, FrameVector2d frameVectorAB)
   {
      Vector2d vectorDA = new Vector2d();
      Vector2d vectorAB = new Vector2d();
      frameVectorDA.get(vectorDA);
      frameVectorAB.get(vectorAB);

      if (AngleTools.angleMinusPiToPi(vectorAB, vectorDA) > 0.0)
      {
         return true;
      }
      else
      {
         return false;
      }      
   }
   
   private void computeJointToJointVectorProjectedOntoFourBarPlane(FrameVector2d vectorBCProjectedToPack, FrameVector2d vectorCDProjectedToPack, FrameVector2d vectorDAProjectedToPack, FrameVector2d vectorABProjectedToPack)
   {
      FramePoint masterJointAPosition = new FramePoint(masterJointA.getFrameAfterJoint());
      FramePoint jointBPosition = new FramePoint(passiveJointB.getFrameAfterJoint());
      FramePoint jointCPosition = new FramePoint(passiveJointC.getFrameAfterJoint());
      FramePoint jointDZeroAnglePosition = new FramePoint(passiveJointD.getFrameAfterJoint());

      if(DEBUG)
         System.out.println("\njointDInJointABeforeFrame = " + jointDInJointABeforeFrame + "\n");

      FramePoint jointDClosedLoopPosition = new FramePoint();
      jointDClosedLoopPosition.setIncludingFrame(masterJointA.getFrameBeforeJoint(), jointDInJointABeforeFrame);

      if(DEBUG)
         System.out.println("\njointDClosedLoopPosition = " + jointDClosedLoopPosition + "\n");

      jointBPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointCPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointDZeroAnglePosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointDClosedLoopPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      masterJointAPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);

      FrameVector vectorAB = new FrameVector(frameBeforeFourBarWithZAlongJointAxis);
      FrameVector vectorBC = new FrameVector(frameBeforeFourBarWithZAlongJointAxis);
      FrameVector vectorCD = new FrameVector(frameBeforeFourBarWithZAlongJointAxis);
      FrameVector vectorDA = new FrameVector(frameBeforeFourBarWithZAlongJointAxis);

      vectorAB.sub(jointBPosition, masterJointAPosition);
      vectorBC.sub(jointCPosition, jointBPosition);
      vectorCD.sub(jointDZeroAnglePosition, jointCPosition);
      vectorDA.sub(masterJointAPosition, jointDClosedLoopPosition);

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

   private FourBarCalculatorWithDerivatives createFourBarCalculator(FrameVector2d vectorBCProjected, FrameVector2d vectorCDProjected, FrameVector2d vectorDAProjected, FrameVector2d vectorABProjected)
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

      FourBarCalculatorWithDerivatives fourBarCalculator = new FourBarCalculatorWithDerivatives(DA, masterLinkAB, BC, CD);

      return fourBarCalculator;
   }

   private void initializeInteriorAnglesAtZeroConfigurationAndJointSigns(FrameVector2d vectorDAProjected, FrameVector2d vectorABProjected, FrameVector2d vectorBCProjected, FrameVector2d vectorCDProjected)
   {
      FrameVector jointBAxis = passiveJointB.getJointAxis();
      FrameVector jointCAxis = passiveJointC.getJointAxis();
      FrameVector jointDAxis = passiveJointD.getJointAxis();

      jointBAxis.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointCAxis.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointDAxis.changeFrame(frameBeforeFourBarWithZAlongJointAxis);

      Vector2d tempVectorAB = new Vector2d();
      Vector2d tempVectorBC = new Vector2d();
      Vector2d tempVectorCD = new Vector2d();
      Vector2d tempVectorDA = new Vector2d();

      vectorABProjected.get(tempVectorAB);
      vectorBCProjected.get(tempVectorBC);
      vectorCDProjected.get(tempVectorCD);
      vectorDAProjected.get(tempVectorDA);

      if (fourBarClockwise)
      {
         interiorAnglesAtZeroConfiguration[0] = Math.PI - AngleTools.angleMinusPiToPi(tempVectorAB, tempVectorDA);
         interiorAnglesAtZeroConfiguration[1] = Math.PI - AngleTools.angleMinusPiToPi(tempVectorBC, tempVectorAB);
         interiorAnglesAtZeroConfiguration[2] = Math.PI - AngleTools.angleMinusPiToPi(tempVectorCD, tempVectorBC);
         interiorAnglesAtZeroConfiguration[3] = Math.PI;

         passiveJointSigns[0] = 1.0;
         passiveJointSigns[1] = jointBAxis.getZ();
         passiveJointSigns[2] = jointCAxis.getZ();
         passiveJointSigns[3] = jointDAxis.getZ();    
      }
      else
      {
         interiorAnglesAtZeroConfiguration[0] = Math.PI - AngleTools.angleMinusPiToPi(tempVectorDA, tempVectorAB);
         interiorAnglesAtZeroConfiguration[1] = Math.PI - AngleTools.angleMinusPiToPi(tempVectorAB, tempVectorBC);
         interiorAnglesAtZeroConfiguration[2] = Math.PI - AngleTools.angleMinusPiToPi(tempVectorBC, tempVectorCD);
         interiorAnglesAtZeroConfiguration[3] = Math.PI;

         passiveJointSigns[0] = - 1.0;
         passiveJointSigns[1] = - jointBAxis.getZ();
         passiveJointSigns[2] = - jointCAxis.getZ();
         passiveJointSigns[3] = - jointDAxis.getZ();    
      }

      if (DEBUG)
      {
         System.out.println("\nOffset angle debugging:");
         System.out.println("offset A = " + interiorAnglesAtZeroConfiguration[0] / Math.PI);
         System.out.println("offset B = " + interiorAnglesAtZeroConfiguration[1] / Math.PI);
         System.out.println("offset C = " + interiorAnglesAtZeroConfiguration[2] / Math.PI);
         System.out.println("offset D = " + interiorAnglesAtZeroConfiguration[3] / Math.PI);
         System.out.println("joint sign A = " + passiveJointSigns[0]);
         System.out.println("joint sign B = " + passiveJointSigns[1]);
         System.out.println("joint sign C = " + passiveJointSigns[2]);
         System.out.println("joint sign D = " + passiveJointSigns[2]);
      }
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
         double interiorAngleBLimit0 = convertJointAngleToInteriorAngle(jointB.getJointLimitLower(), 1);
         double interiorAngleBLimit1 = convertJointAngleToInteriorAngle(jointB.getJointLimitUpper(), 1);
         double maxAngleB = Math.max(interiorAngleBLimit0, interiorAngleBLimit1);

         if (MathTools.isInsideBoundsExclusive(maxAngleB, 0.0, Math.PI))
         {
            fourBarCalculator.computeMasterJointAngleGivenAngleABC(maxAngleB);
            minValidMasterJointAngle = Math.max(minValidMasterJointAngle, fourBarCalculator.getAngleDAB());
         }
      }

      if (jointC.getJointLimitLower() != Double.NEGATIVE_INFINITY)
      {
         double interiorAngleCLimit0 = convertJointAngleToInteriorAngle(jointC.getJointLimitLower(), 2);
         double interiorAngleCLimit1 = convertJointAngleToInteriorAngle(jointC.getJointLimitUpper(), 2);
         double minAngleC = Math.min(interiorAngleCLimit0, interiorAngleCLimit1);

         if (MathTools.isInsideBoundsExclusive(minAngleC, 0.0, Math.PI))
         {
            fourBarCalculator.computeMasterJointAngleGivenAngleBCD(minAngleC);
            minValidMasterJointAngle = Math.max(minValidMasterJointAngle, fourBarCalculator.getAngleDAB());
         }
      }

      if (jointD.getJointLimitUpper() != Double.POSITIVE_INFINITY)
      {
         double interiorAngleDLimit0 = convertJointAngleToInteriorAngle(jointD.getJointLimitLower(), 3);
         double interiorAngleDLimit1 = convertJointAngleToInteriorAngle(jointD.getJointLimitUpper(), 3);
         double maxAngleD = Math.max(interiorAngleDLimit0, interiorAngleDLimit1);

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
         double interiorAngleBLimit0 = convertJointAngleToInteriorAngle(jointB.getJointLimitLower(), 1);
         double interiorAngleBLimit1 = convertJointAngleToInteriorAngle(jointB.getJointLimitUpper(), 1);
         double minAngleB = Math.min(interiorAngleBLimit0, interiorAngleBLimit1);

         if (MathTools.isInsideBoundsExclusive(minAngleB, 0.0, Math.PI))
         {
            fourBarCalculator.computeMasterJointAngleGivenAngleABC(minAngleB);
            maxValidMasterJointAngle = Math.min(maxValidMasterJointAngle, fourBarCalculator.getAngleDAB());
         }
      }

      if (jointC.getJointLimitUpper() != Double.POSITIVE_INFINITY)
      {
         double interiorAngleCLimit0 = convertJointAngleToInteriorAngle(jointC.getJointLimitLower(), 2);
         double interiorAngleCLimit1 = convertJointAngleToInteriorAngle(jointC.getJointLimitUpper(), 2);
         double maxAngleC = Math.max(interiorAngleCLimit0, interiorAngleCLimit1);

         if (MathTools.isInsideBoundsExclusive(maxAngleC, 0.0, Math.PI))
         {
            fourBarCalculator.computeMasterJointAngleGivenAngleBCD(maxAngleC);
            maxValidMasterJointAngle = Math.min(maxValidMasterJointAngle, fourBarCalculator.getAngleDAB());
         }
      }

      if (jointD.getJointLimitLower() != Double.NEGATIVE_INFINITY)
      {
         double interiorAngleDLimit0 = convertJointAngleToInteriorAngle(jointD.getJointLimitLower(), 3);
         double interiorAngleDLimit1 = convertJointAngleToInteriorAngle(jointD.getJointLimitUpper(), 3);
         double minAngleD = Math.min(interiorAngleDLimit0, interiorAngleDLimit1);

         if (MathTools.isInsideBoundsExclusive(minAngleD, 0.0, Math.PI))
         {
            fourBarCalculator.computeMasterJointAngleGivenAngleCDA(minAngleD);
            maxValidMasterJointAngle = Math.min(maxValidMasterJointAngle, fourBarCalculator.getAngleDAB());
         }
      }

      return maxValidMasterJointAngle;
   }

   public void update()
   {
      double currentMasterJointA = masterJointA.getQ();
      double interiorAngleA = convertJointAngleToInteriorAngle(currentMasterJointA, 0);

      if(DEBUG)
         System.out.println("interior angle A = " + interiorAngleA / Math.PI);

      if (currentMasterJointA < masterJointA.getJointLimitLower() || currentMasterJointA > masterJointA.getJointLimitUpper())
      {
         throw new RuntimeException(masterJointA.getName() + " is set outside of its bounds [" + masterJointA.getJointLimitLower() + ", " + masterJointA.getJointLimitUpper() + "]. The current value is: " +  masterJointA.getQ());
      }

      fourBarCalculator.updateAnglesVelocitiesAndAccelerationsGivenAngleDAB(interiorAngleA, masterJointA.getQd(), masterJointA.getQdd());
      jacobian = fourBarJacobianSolver.computeJacobian(fourBarOutputJoint);
      
      if(DEBUG)
      {
         System.out.println("\nJacobian: \n" + jacobian);
      }
      
      outputJointVelocitiesToPack = jacobian;
      fourBarJacobianSolver.solveLinearVelFromAngularVel(outputJointVelocitiesToPack, masterJointA.getQd());
     
      if(DEBUG)
      {
         System.out.println("\nVelocity output (J*qd): \n" + outputJointVelocitiesToPack);

         System.out.println("Calculator output:");
         System.out.println("ABC = " + fourBarCalculator.getAngleABC());
         System.out.println("BCD = " + fourBarCalculator.getAngleBCD());
         System.out.println("CDA = " + fourBarCalculator.getAngleCDA());
         System.out.println("DtABC = " + fourBarCalculator.getAngleDtABC());
         System.out.println("DtBCD = " + fourBarCalculator.getAngleDtBCD());
         System.out.println("DtCDA = " + fourBarCalculator.getAngleDtCDA());
      }

      passiveJointB.setQ(convertInteriorAngleToJointAngle(fourBarCalculator.getAngleABC(), 1));
      passiveJointC.setQ(convertInteriorAngleToJointAngle(fourBarCalculator.getAngleBCD(), 2));
      passiveJointD.setQ(convertInteriorAngleToJointAngle(fourBarCalculator.getAngleCDA(), 3));

      passiveJointB.setQd(passiveJointSigns[1] * fourBarCalculator.getAngleDtABC());
      passiveJointC.setQd(passiveJointSigns[2] * fourBarCalculator.getAngleDtBCD());
      passiveJointD.setQd(passiveJointSigns[3] * fourBarCalculator.getAngleDtCDA());

      passiveJointB.setQdd(passiveJointSigns[1] * fourBarCalculator.getAngleDt2ABC());
      passiveJointC.setQdd(passiveJointSigns[2] * fourBarCalculator.getAngleDt2BCD());
      passiveJointD.setQdd(passiveJointSigns[3] * fourBarCalculator.getAngleDt2CDA());
   }

   /**
    * @param interiorAngle interior angle of the joint when viewing the fourbar as a quadrilateral
    * @param passiveJointIndex 0->A, 1->B, 2->C, 3->D
    * @return
    */
   private double convertInteriorAngleToJointAngle(double interiorAngle, int passiveJointIndex)
   {
      return passiveJointSigns[passiveJointIndex] * (interiorAngle - interiorAnglesAtZeroConfiguration[passiveJointIndex]);
   }

   /**
    * @param jointAngle interior angle of the joint when viewing the fourbar as a quadrilateral
    * @param passiveJointIndex 0->A, 1->B, 2->C, 3->D
    * @return
    */
   private double convertJointAngleToInteriorAngle(double jointAngle, int passiveJointIndex)
   {
      return interiorAnglesAtZeroConfiguration[passiveJointIndex] + passiveJointSigns[passiveJointIndex] * jointAngle;
   }

   private InverseDynamicsJoint[] getJointsForJacobianCalculation(PassiveRevoluteJoint outputJoint)
   {
      InverseDynamicsJoint[] joints;
      
      if (fourBarOutputJoint == passiveJointD)
      {      
         joints = new InverseDynamicsJoint[3] ;
         joints[0] = masterJointA;
         joints[1] = passiveJointB;
         joints[2] = passiveJointC;
      }      
      else if (fourBarOutputJoint == passiveJointC)
      {      
         joints = new InverseDynamicsJoint[2] ;
         joints[0] = masterJointA;
         joints[1] = passiveJointB;
      }    
      else
      {
         joints = new InverseDynamicsJoint[1] ;
         joints[0] = masterJointA;
      }
      
      return joints;
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
   
   public DenseMatrix64F getOutputJointLinearVelocities()
   {
      return outputJointVelocitiesToPack;
   }
   
   public DenseMatrix64F getFourBarJacobian()
   {
      return jacobian;
   }
}
