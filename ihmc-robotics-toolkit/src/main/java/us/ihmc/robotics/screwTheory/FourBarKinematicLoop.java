package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.Collections;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.kinematics.fourbar.ConstantSideFourBarCalculatorWithDerivatives;

public class FourBarKinematicLoop
{
   /*
    * Representation of the four bar with name correspondences. This name
    * convention matches the one used in the ConstantSideFourBarCalculatorWithDerivatives
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

   private static final boolean DEBUG = false;
   private static final double eps = 1e-7;

   private final String name;
   private final RevoluteJoint masterJointA;
   private final PassiveRevoluteJoint passiveJointB, passiveJointC, passiveJointD, fourBarOutputJoint;
   private final Vector3D jointDInJointABeforeFrame;

   private final ConstantSideFourBarCalculatorWithDerivatives fourBarCalculator;

   private final FourBarKinematicLoopJacobianSolver fourBarJacobianSolver;
   private final DenseMatrix64F columnJacobian = new DenseMatrix64F(6, 1);

   private final double[] interiorAnglesAtZeroConfiguration = new double[4];
   private final int[] jointSigns = new int[4];

   private final ReferenceFrame frameBeforeFourBarWithZAlongJointAxis;

   private final boolean fourBarIsClockwise;

   /**
    * @param name
    * @param masterJointA
    * @param passiveJointB
    * @param passiveJointC
    * @param passiveJointD
    * @param jointDInJointABeforeFrame The position in A's frame to which D is rigidly attached
    * @param recomputeJointLimits
    *
    * <dt><b>Precondition:</b><dd>
    * The cartesian positions of masterJointA, passiveJointB, passiveJointC, and passiveJointD must form a convex quadrilateral when their joint angles are 0. <br>
    * Additionally, the cartesian positions of masterJointA, passiveJointB, passiveJointC and jointDInJointABeforeFrame must form a convex quadrilateral when their joint angles are 0
    *
    */
   public FourBarKinematicLoop(String name, RevoluteJoint masterJointA, PassiveRevoluteJoint passiveJointB, PassiveRevoluteJoint passiveJointC,
         PassiveRevoluteJoint passiveJointD, Vector3DReadOnly jointDInJointABeforeFrame, boolean recomputeJointLimits)
   {
      this.name = name;
      this.masterJointA = masterJointA;
      this.passiveJointB = passiveJointB;
      this.passiveJointC = passiveJointC;
      this.passiveJointD = passiveJointD;
      this.jointDInJointABeforeFrame = new Vector3D(jointDInJointABeforeFrame);

      // Rotation axis
      FrameVector3D masterJointAxis = masterJointA.getJointAxis();
      masterJointAxis.changeFrame(masterJointA.getFrameBeforeJoint());
      frameBeforeFourBarWithZAlongJointAxis = GeometryTools
            .constructReferenceFrameFromPointAndAxis(name + "FrameWithZAlongJointAxis", new FramePoint3D(masterJointA.getFrameBeforeJoint()), Axis.Z,
                  masterJointAxis);

      FrameVector3D masterAxis = masterJointA.getJointAxis();
      FrameVector3D jointBAxis = passiveJointB.getJointAxis();
      FrameVector3D jointCAxis = passiveJointC.getJointAxis();
      FrameVector3D jointDAxis = passiveJointD.getJointAxis();
      FourBarKinematicLoopTools.checkJointAxesAreParallel(masterAxis, jointBAxis, jointCAxis, jointDAxis);

      // Joint order
      FourBarKinematicLoopTools.checkCorrectJointOrder(name, masterJointA, passiveJointB, passiveJointC, passiveJointD);

      // Go to zero configuration
      masterJointA.setQ(0.0);
      passiveJointB.setQ(0.0);
      passiveJointC.setQ(0.0);
      passiveJointD.setQ(0.0);

      // Link lengths
      FrameVector2D vectorBCProjected = new FrameVector2D();
      FrameVector2D vectorCDProjected = new FrameVector2D();
      FrameVector2D vectorDAProjected = new FrameVector2D();
      FrameVector2D vectorABProjected = new FrameVector2D();
      FrameVector2D vectorDAClosurePointProjected = new FrameVector2D();
      FrameVector2D vectorCDClosurePointProjected = new FrameVector2D();
      computeJointToJointVectorProjectedOntoFourBarPlane(vectorBCProjected, vectorCDProjected, vectorDAProjected, vectorABProjected,
            vectorDAClosurePointProjected, vectorCDClosurePointProjected);

      // Check that the fourbar is convex in its zero angle configuration and it's orientation (CW or CCW)
      FourBarKinematicLoopTools
            .checkFourBarConvexityAndOrientation(vectorABProjected, vectorBCProjected, vectorCDClosurePointProjected, vectorDAClosurePointProjected);
      fourBarIsClockwise = FourBarKinematicLoopTools
            .checkFourBarConvexityAndOrientation(vectorABProjected, vectorBCProjected, vectorCDProjected, vectorDAProjected);

      // Calculator
      fourBarCalculator = createFourBarCalculator(vectorBCProjected, vectorCDProjected, vectorDAClosurePointProjected, vectorABProjected);

      // Set output joint     
      fourBarOutputJoint = FourBarKinematicLoopTools.getFourBarOutputJoint(passiveJointB, passiveJointC, passiveJointD);

      // Jacobian
      fourBarJacobianSolver = new FourBarKinematicLoopJacobianSolver(fourBarOutputJoint);

      // Initialize interior angle offsets and signs
      initializeInteriorAnglesAtZeroConfigurationAndJointSigns(vectorDAClosurePointProjected, vectorABProjected, vectorBCProjected, vectorCDProjected);

      // Find the most conservative limits for the master joint angle (A) and reset them if necessary
      if (recomputeJointLimits)
      {
         /*
          * - If the limits for B, C, and/or D are given and are more restrictive than those of A crop the value of Amax and/or Amin. 
          * - Else if the limits given for A are the most restrictive of all, keep them. 
          * - Else set the limits to the value given by the calculator.
          */
         double[] masterJointBounds = computeMinAndMaxValidMasterJointAngles(masterJointA, passiveJointB, passiveJointC, passiveJointD);

         masterJointA.setJointLimitLower(masterJointBounds[0]);
         masterJointA.setJointLimitUpper(masterJointBounds[1]);
      }
      else
      {
         verifyMasterJointLimits();
      }

      if (DEBUG)
      {
         System.out.println("\nOutput joint: " + fourBarOutputJoint.name + "\n");
         double qA = masterJointA.getQ();
         double qB = passiveJointB.getQ();
         double qC = passiveJointC.getQ();
         double qD = passiveJointD.getQ();
         System.out.println("\nInitial joint angles debugging:\n\n" + "MasterQ: " + qA + "\njointBQ: " + qB + "\njointCQ: " + qC + "\njointDQ: " + qD + "\n");
         System.out.println("\nMax master joint angle: " + masterJointA.getJointLimitUpper());
         System.out.println("Min master joint angle: " + masterJointA.getJointLimitLower());
      }
   }

   private void computeJointToJointVectorProjectedOntoFourBarPlane(FrameVector2D vectorBCProjectedToPack, FrameVector2D vectorCDProjectedToPack,
         FrameVector2D vectorDAProjectedToPack, FrameVector2D vectorABProjectedToPack, FrameVector2D vectorDAClosurePointProjectedToPack,
         FrameVector2D vectorCDClosurePointProjectedToPack)
   {
      FramePoint3D masterJointAPosition = new FramePoint3D(masterJointA.getFrameAfterJoint());
      FramePoint3D jointBPosition = new FramePoint3D(passiveJointB.getFrameAfterJoint());
      FramePoint3D jointCPosition = new FramePoint3D(passiveJointC.getFrameAfterJoint());
      FramePoint3D jointDZeroAnglePosition = new FramePoint3D(passiveJointD.getFrameAfterJoint());
      FramePoint3D jointDClosedLoopPosition = new FramePoint3D(masterJointA.getFrameBeforeJoint(), jointDInJointABeforeFrame);

      jointBPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointCPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointDZeroAnglePosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointDClosedLoopPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      masterJointAPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);

      FrameVector3D vectorAB = new FrameVector3D(frameBeforeFourBarWithZAlongJointAxis);
      FrameVector3D vectorBC = new FrameVector3D(frameBeforeFourBarWithZAlongJointAxis);
      FrameVector3D vectorCD = new FrameVector3D(frameBeforeFourBarWithZAlongJointAxis);
      FrameVector3D vectorDA = new FrameVector3D(frameBeforeFourBarWithZAlongJointAxis);
      FrameVector3D vectorDAClosurePoint = new FrameVector3D(frameBeforeFourBarWithZAlongJointAxis);
      FrameVector3D vectorCDClosurePoint = new FrameVector3D(frameBeforeFourBarWithZAlongJointAxis);

      vectorAB.sub(jointBPosition, masterJointAPosition);
      vectorBC.sub(jointCPosition, jointBPosition);
      vectorCD.sub(jointDZeroAnglePosition, jointCPosition);
      vectorDA.sub(masterJointAPosition, jointDZeroAnglePosition);
      vectorDAClosurePoint.sub(masterJointAPosition, jointDClosedLoopPosition);
      vectorCDClosurePoint.sub(jointDClosedLoopPosition, jointCPosition);

      vectorBCProjectedToPack.setIncludingFrame(vectorBC);
      vectorCDProjectedToPack.setIncludingFrame(vectorCD);
      vectorDAProjectedToPack.setIncludingFrame(vectorDA);
      vectorABProjectedToPack.setIncludingFrame(vectorAB);
      vectorDAClosurePointProjectedToPack.setIncludingFrame(vectorDAClosurePoint);
      vectorCDClosurePointProjectedToPack.setIncludingFrame(vectorCDClosurePoint);

      if (DEBUG)
      {
         System.out.println("\njointDInJointABeforeFrame = " + jointDInJointABeforeFrame + "\n");
         System.out.println("\njointDClosedLoopPosition = " + jointDClosedLoopPosition + "\n");
         System.out.println("\nJoint to joint vectors debugging:\n");
         System.out.println("vector ab = " + vectorAB);
         System.out.println("vector bc = " + vectorBC);
         System.out.println("vector cd = " + vectorCD);
         System.out.println("vector da = " + vectorDA);
      }
   }

   private ConstantSideFourBarCalculatorWithDerivatives createFourBarCalculator(FrameVector2D vectorBCProjected, FrameVector2D vectorCDProjected,
         FrameVector2D vectorDAProjected, FrameVector2D vectorABProjected)
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

      ConstantSideFourBarCalculatorWithDerivatives fourBarCalculator = new ConstantSideFourBarCalculatorWithDerivatives(DA, masterLinkAB, BC, CD);

      return fourBarCalculator;
   }

   private void initializeInteriorAnglesAtZeroConfigurationAndJointSigns(FrameVector2D vectorDAProjected, FrameVector2D vectorABProjected,
         FrameVector2D vectorBCProjected, FrameVector2D vectorCDProjected)
   {
      FrameVector3D jointBAxis = passiveJointB.getJointAxis();
      FrameVector3D jointCAxis = passiveJointC.getJointAxis();
      FrameVector3D jointDAxis = passiveJointD.getJointAxis();

      jointBAxis.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointCAxis.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointDAxis.changeFrame(frameBeforeFourBarWithZAlongJointAxis);

      if (fourBarIsClockwise)
      {
         interiorAnglesAtZeroConfiguration[0] = Math.PI - AngleTools.angleMinusPiToPi(vectorABProjected, vectorDAProjected);
         interiorAnglesAtZeroConfiguration[1] = Math.PI - AngleTools.angleMinusPiToPi(vectorBCProjected, vectorABProjected);
         interiorAnglesAtZeroConfiguration[2] = Math.PI - AngleTools.angleMinusPiToPi(vectorCDProjected, vectorBCProjected);
         interiorAnglesAtZeroConfiguration[3] = Math.PI;

         jointSigns[0] = 1;
         jointSigns[1] = (int) Math.round(jointBAxis.getZ());
         jointSigns[2] = (int) Math.round(jointCAxis.getZ());
         jointSigns[3] = (int) Math.round(jointDAxis.getZ());
      }
      else
      {
         interiorAnglesAtZeroConfiguration[0] = Math.PI - AngleTools.angleMinusPiToPi(vectorDAProjected, vectorABProjected);
         interiorAnglesAtZeroConfiguration[1] = Math.PI - AngleTools.angleMinusPiToPi(vectorABProjected, vectorBCProjected);
         interiorAnglesAtZeroConfiguration[2] = Math.PI - AngleTools.angleMinusPiToPi(vectorBCProjected, vectorCDProjected);
         interiorAnglesAtZeroConfiguration[3] = Math.PI;

         jointSigns[0] = -1;
         jointSigns[1] = -(int) Math.round(jointBAxis.getZ());
         jointSigns[2] = -(int) Math.round(jointCAxis.getZ());
         jointSigns[3] = -(int) Math.round(jointDAxis.getZ());
      }

      if (DEBUG)
      {
         System.out.println("\nOffset angle debugging:");
         System.out.println("offset A = " + interiorAnglesAtZeroConfiguration[0] / Math.PI);
         System.out.println("offset B = " + interiorAnglesAtZeroConfiguration[1] / Math.PI);
         System.out.println("offset C = " + interiorAnglesAtZeroConfiguration[2] / Math.PI);
         System.out.println("offset D = " + interiorAnglesAtZeroConfiguration[3] / Math.PI);
         System.out.println("joint sign A = " + jointSigns[0]);
         System.out.println("joint sign B = " + jointSigns[1]);
         System.out.println("joint sign C = " + jointSigns[2]);
         System.out.println("joint sign D = " + jointSigns[3]);
      }
   }

   /**
    * Clips the min/max master joint angles if the lower/upper limit for any of the joints that are passed in is more restrictive
    */
   private double[] computeMinAndMaxValidMasterJointAngles(RevoluteJoint masterJointA, PassiveRevoluteJoint jointB, PassiveRevoluteJoint jointC,
         PassiveRevoluteJoint jointD)
   {
      ArrayList<Double> lowerBounds = new ArrayList<Double>();
      ArrayList<Double> upperBounds = new ArrayList<Double>();

      // bounds from the max and min quadrilateral angles
      double masterLimitFromQuadrilateralMin = convertInteriorAngleToJointAngle(fourBarCalculator.getMinDAB(), 0);
      double masterLimitFromQuadrilateralMax = convertInteriorAngleToJointAngle(fourBarCalculator.getMaxDAB(), 0);

      if (jointSigns[0] > 0)
      {
         lowerBounds.add(masterLimitFromQuadrilateralMin);
         upperBounds.add(masterLimitFromQuadrilateralMax);
      }
      else
      {
         lowerBounds.add(masterLimitFromQuadrilateralMax);
         upperBounds.add(masterLimitFromQuadrilateralMin);
      }

      addJointLimitToBoundsIfValid(lowerBounds, upperBounds, masterJointA.getJointLimitLower(), 0, false);
      addJointLimitToBoundsIfValid(lowerBounds, upperBounds, masterJointA.getJointLimitUpper(), 0, true);
      addJointLimitToBoundsIfValid(lowerBounds, upperBounds, jointB.getJointLimitLower(), 1, false);
      addJointLimitToBoundsIfValid(lowerBounds, upperBounds, jointB.getJointLimitUpper(), 1, true);
      addJointLimitToBoundsIfValid(lowerBounds, upperBounds, jointC.getJointLimitLower(), 2, false);
      addJointLimitToBoundsIfValid(lowerBounds, upperBounds, jointC.getJointLimitUpper(), 2, true);
      addJointLimitToBoundsIfValid(lowerBounds, upperBounds, jointD.getJointLimitLower(), 3, false);
      addJointLimitToBoundsIfValid(lowerBounds, upperBounds, jointD.getJointLimitUpper(), 3, true);

      Collections.sort(lowerBounds);
      Collections.reverse(lowerBounds);
      Collections.sort(upperBounds);

      return new double[] {lowerBounds.get(0), upperBounds.get(0)};
   }

   private void addJointLimitToBoundsIfValid(ArrayList<Double> lowerBoundsList, ArrayList<Double> upperBoundsList, double jointAngleLimit, int jointIndex,
         boolean isAnUpperBound)
   {
      if (jointAngleLimit != Double.POSITIVE_INFINITY && jointAngleLimit != Double.NEGATIVE_INFINITY)
      {
         double interiorAngle = convertJointAngleToInteriorAngle(jointAngleLimit, jointIndex);
         double masterJointLimit;

         if (!MathTools.intervalContains(interiorAngle, 0.0, Math.PI, false, false))
            return;

         switch (jointIndex)
         {
         case 0:
            masterJointLimit = convertInteriorAngleToJointAngle(interiorAngle, 0);
            break;
         case 1:
            fourBarCalculator.computeMasterJointAngleGivenAngleABC(interiorAngle);
            masterJointLimit = convertInteriorAngleToJointAngle(fourBarCalculator.getAngleDAB(), 0);
            break;
         case 2:
            fourBarCalculator.computeMasterJointAngleGivenAngleBCD(interiorAngle);
            masterJointLimit = convertInteriorAngleToJointAngle(fourBarCalculator.getAngleDAB(), 0);
            break;
         case 3:
            fourBarCalculator.computeMasterJointAngleGivenAngleCDA(interiorAngle);
            masterJointLimit = convertInteriorAngleToJointAngle(fourBarCalculator.getAngleDAB(), 0);
            break;
         default:
            throw new RuntimeException("Invalid joint index " + jointIndex + " in four bar kinematic loop");
         }

         boolean isAMasterJointUpperBound = isAnUpperBound ^ !jointMaxCorrespondsToMasterJointMax(jointIndex);
         if (isAMasterJointUpperBound)
            upperBoundsList.add(masterJointLimit);
         else
            lowerBoundsList.add(masterJointLimit);
      }
   }

   private boolean jointMaxCorrespondsToMasterJointMax(int jointIndex)
   {
      boolean positiveMasterJointCoeff = jointSigns[0] > 0;
      boolean positivePassiveJointCoeff = jointSigns[jointIndex] > 0;
      boolean overallSignChangeIsNegativeOne = positiveMasterJointCoeff ^ positivePassiveJointCoeff;

      // when b or d have a max interior angle, a has a min interior angle
      if (jointIndex == 1 || jointIndex == 3)
         return overallSignChangeIsNegativeOne;
      else
         return !overallSignChangeIsNegativeOne;
   }

   public void verifyMasterJointLimits()
   {
      double maxValidMasterJointAngle = fourBarIsClockwise ?
            convertInteriorAngleToJointAngle(fourBarCalculator.getMaxDAB(), 0) :
            convertInteriorAngleToJointAngle(fourBarCalculator.getMinDAB(), 0);
      double minValidMasterJointAngle = fourBarIsClockwise ?
            convertInteriorAngleToJointAngle(fourBarCalculator.getMinDAB(), 0) :
            convertInteriorAngleToJointAngle(fourBarCalculator.getMaxDAB(), 0);

      // A) Angle limits not set
      if (masterJointA.getJointLimitLower() == Double.NEGATIVE_INFINITY || masterJointA.getJointLimitUpper() == Double.POSITIVE_INFINITY)
      {
         throw new RuntimeException(
               "Must set the joint limits for the master joint of the " + name + " four bar.\nNote that for the given link lengths max angle is "
                     + maxValidMasterJointAngle + "and min angle is" + minValidMasterJointAngle);
      }

      // B) Max angle limit is too large
      if (masterJointA.getJointLimitUpper() > maxValidMasterJointAngle + eps)
      {
         throw new RuntimeException("The maximum valid joint angle for the master joint of the " + name + " four bar is " + maxValidMasterJointAngle
               + " to avoid flipping, but was set to " + masterJointA.getJointLimitUpper());
      }

      // C) Min angle limit is too small
      if (masterJointA.getJointLimitLower() < minValidMasterJointAngle - eps)
      {
         throw new RuntimeException("The minimum valid joint angle for the master joint of the " + name + " four bar is " + minValidMasterJointAngle
               + " to avoid flipping, but was set to " + masterJointA.getJointLimitLower());
      }
   }

   public void update()
   {
      double currentMasterJointA = masterJointA.getQ();
      double interiorAngleA = convertJointAngleToInteriorAngle(currentMasterJointA, 0);
      double interiorAngleDtA = jointSigns[0] * masterJointA.getQd();
      double interiorAngleDt2A = jointSigns[0] * masterJointA.getQdd();

      if (currentMasterJointA < masterJointA.getJointLimitLower() || currentMasterJointA > masterJointA.getJointLimitUpper())
      {
         throw new RuntimeException(
               masterJointA.getName() + " is set outside of its bounds [" + masterJointA.getJointLimitLower() + ", " + masterJointA.getJointLimitUpper()
                     + "]. The current value is: " + masterJointA.getQ());
      }

      fourBarCalculator.updateAnglesVelocitiesAndAccelerationsGivenAngleDAB(interiorAngleA, interiorAngleDtA, interiorAngleDt2A);

      passiveJointB.setQ(convertInteriorAngleToJointAngle(fourBarCalculator.getAngleABC(), 1));
      passiveJointC.setQ(convertInteriorAngleToJointAngle(fourBarCalculator.getAngleBCD(), 2));
      passiveJointD.setQ(convertInteriorAngleToJointAngle(fourBarCalculator.getAngleCDA(), 3));

      passiveJointB.setQd(jointSigns[1] * fourBarCalculator.getAngleDtABC());
      passiveJointC.setQd(jointSigns[2] * fourBarCalculator.getAngleDtBCD());
      passiveJointD.setQd(jointSigns[3] * fourBarCalculator.getAngleDtCDA());

      passiveJointB.setQdd(jointSigns[1] * fourBarCalculator.getAngleDt2ABC());
      passiveJointC.setQdd(jointSigns[2] * fourBarCalculator.getAngleDt2BCD());
      passiveJointD.setQdd(jointSigns[3] * fourBarCalculator.getAngleDt2CDA());

      fourBarJacobianSolver.getUnitJacobian(columnJacobian);
      CommonOps.scale(masterJointA.getQd(), columnJacobian);

      if (DEBUG)
      {
         System.out.println("interior angle A = " + interiorAngleA / Math.PI);
         System.out.println("\nVelocity output (J*qd): \n" + columnJacobian);
         System.out.println("\nJacobian: \n" + columnJacobian);

         System.out.println("Calculator output:");
         System.out.println("ABC = " + fourBarCalculator.getAngleABC());
         System.out.println("BCD = " + fourBarCalculator.getAngleBCD());
         System.out.println("CDA = " + fourBarCalculator.getAngleCDA());
         System.out.println("DtABC = " + fourBarCalculator.getAngleDtABC());
         System.out.println("DtBCD = " + fourBarCalculator.getAngleDtBCD());
         System.out.println("DtCDA = " + fourBarCalculator.getAngleDtCDA());
      }
   }

   /**
    * @param interiorAngle interior angle of the joint when viewing the fourbar as a quadrilateral
    * @param passiveJointIndex 0->A, 1->B, 2->C, 3->D
    * @return
    */
   private double convertInteriorAngleToJointAngle(double interiorAngle, int passiveJointIndex)
   {
      return jointSigns[passiveJointIndex] * (interiorAngle - interiorAnglesAtZeroConfiguration[passiveJointIndex]);
   }

   private double convertInteriorAngularDerivativeToJointAngularDerivative(double interiorAngularDerivative, int passiveJointIndex)
   {
      return jointSigns[passiveJointIndex] * interiorAngularDerivative;
   }

   /**
    * @param jointAngle interior angle of the joint when viewing the fourbar as a quadrilateral
    * @param passiveJointIndex 0->A, 1->B, 2->C, 3->D
    * @return
    */
   private double convertJointAngleToInteriorAngle(double jointAngle, int passiveJointIndex)
   {
      return interiorAnglesAtZeroConfiguration[passiveJointIndex] + jointSigns[passiveJointIndex] * jointAngle;
   }

   private double convertJointAngularDerivativeToInteriorAngularDerivative(double jointAngularDerivative, int passiveJointIndex)
   {
      return jointSigns[passiveJointIndex] * jointAngularDerivative;
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

   public PassiveRevoluteJoint getFourBarOutputJoint()
   {
      return fourBarOutputJoint;
   }

   public void getJacobian(DenseMatrix64F jacobianToPack)
   {
      jacobianToPack.set(columnJacobian);
   }

   private class FourBarKinematicLoopJacobianSolver
   {
      private final GeometricJacobian geometricJacobian;
      private final InverseDynamicsJoint[] jointsForJacobianCalculation;
      private final ReferenceFrame geometricJacobianFrame;
      private final DenseMatrix64F geometricJacobianToColumnJacobian;

      public FourBarKinematicLoopJacobianSolver(PassiveRevoluteJoint outputJoint)
      {
         this.jointsForJacobianCalculation = getJointsForJacobianCalculation();
         this.geometricJacobianToColumnJacobian = createGeometricJacobianToColumnJacobianMatrix(jointsForJacobianCalculation);
         this.geometricJacobianFrame = outputJoint.getFrameAfterJoint();
         this.geometricJacobian = new GeometricJacobian(jointsForJacobianCalculation, geometricJacobianFrame);
      }

      private InverseDynamicsJoint[] getJointsForJacobianCalculation()
      {
         InverseDynamicsJoint[] joints;

         if (fourBarOutputJoint == passiveJointD)
         {
            joints = new InverseDynamicsJoint[3];
            joints[0] = masterJointA;
            joints[1] = passiveJointB;
            joints[2] = passiveJointC;
         }
         else if (fourBarOutputJoint == passiveJointC)
         {
            joints = new InverseDynamicsJoint[2];
            joints[0] = masterJointA;
            joints[1] = passiveJointB;
         }
         else
         {
            joints = new InverseDynamicsJoint[1];
            joints[0] = masterJointA;
         }

         return joints;
      }

      public void getUnitJacobian(DenseMatrix64F unitJacobianToPack)
      {
         // Geometric Jacobian - open kinematic chain Jacobian
         geometricJacobian.compute();

         // Vector to go from open loop to closed loop Jacobian
         computeVectorTransformGeometricToColumnJacobian();

         // Column Jacobian - fourbars are a 1DOF system so the Jacobian is a column vector
         CommonOps.mult(geometricJacobian.getJacobianMatrix(), geometricJacobianToColumnJacobian, unitJacobianToPack);
      }

      private void computeVectorTransformGeometricToColumnJacobian()
      {
         // Vector containing angular velocity of passive joints for angular velocity of input joint (master) equal 1
         double masterInteriorAngle = convertJointAngleToInteriorAngle(masterJointA.getQ(), 0);
         double dqA_dqA = 1.0;
         double dInteriorA_dqA = convertJointAngularDerivativeToInteriorAngularDerivative(dqA_dqA, 0);
         fourBarCalculator.updateAnglesAndVelocitiesGivenAngleDAB(masterInteriorAngle, dInteriorA_dqA);
         double dqB_dqA = convertInteriorAngularDerivativeToJointAngularDerivative(fourBarCalculator.getAngleDtABC(), 1);
         double dqC_dqA = convertInteriorAngularDerivativeToJointAngularDerivative(fourBarCalculator.getAngleDtBCD(), 2);

         if (jointsForJacobianCalculation.length == 3) // Output joint is D
         {
            geometricJacobianToColumnJacobian.set(3, 1, true, new double[] {dqA_dqA, dqB_dqA, dqC_dqA});
         }
         else if (jointsForJacobianCalculation.length == 2) // Output joint is C
         {
            geometricJacobianToColumnJacobian.set(2, 1, true, new double[] {dqA_dqA, dqB_dqA});
         }
         else // Output joint is B
         {
            geometricJacobianToColumnJacobian.set(1, 1, true, new double[] {dqA_dqA});
         }
      }

      private DenseMatrix64F createGeometricJacobianToColumnJacobianMatrix(InverseDynamicsJoint[] jointsForJacobianCalculation)
      {
         int numberOfJointsForJacobianCalculation = jointsForJacobianCalculation.length;

         if (numberOfJointsForJacobianCalculation < 1 || numberOfJointsForJacobianCalculation > 3)
         {
            throw new RuntimeException(
                  "Illegal number of joints for jacobian calculation. Expected 1, 2, or 3 and got " + numberOfJointsForJacobianCalculation);
         }

         return new DenseMatrix64F(3, jointsForJacobianCalculation.length);
      }
   }
}
