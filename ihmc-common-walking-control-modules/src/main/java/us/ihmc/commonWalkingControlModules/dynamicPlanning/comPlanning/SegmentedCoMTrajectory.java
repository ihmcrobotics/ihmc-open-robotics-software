package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public class SegmentedCoMTrajectory
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final DenseMatrix64F coefficientMultipliers = new DenseMatrix64F(8, 8);
   private final DenseMatrix64F coefficientMultipliersInv = new DenseMatrix64F(8, 8);
   private final DenseMatrix64F xCoefficientConstants = new DenseMatrix64F(8, 1);
   private final DenseMatrix64F yCoefficientConstants = new DenseMatrix64F(8, 1);
   private final DenseMatrix64F zCoefficientConstants = new DenseMatrix64F(8, 1);
   private final DenseMatrix64F xCoefficientVector = new DenseMatrix64F(8, 1);
   private final DenseMatrix64F yCoefficientVector = new DenseMatrix64F(8, 1);
   private final DenseMatrix64F zCoefficientVector = new DenseMatrix64F(8, 1);

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);

   private final FixedFramePoint3DBasics desiredComPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredComVelocity = new FrameVector3D(worldFrame);
   private final FixedFrameVector3DBasics desiredComAcceleration = new FrameVector3D(worldFrame);

   private double startTimeInPhase;
   private double midpointTimeInPhase;
   private double finalTimeInPhase;

   private double omega;
   private double gravityZ;

   private ContactState firstContactState;
   private ContactState secondContactState;

   public void set(double startTimeInPhase, double midpointTimeInPhase, double finalTimeInPhase, double omega, double gravityZ, ContactState firstContactState,
                   ContactState secondContactState, FramePoint3DReadOnly initialCoMPosition, FrameVector3DReadOnly initialCoMVelocity,
                   FramePoint3DReadOnly finalCoMPosition, FrameVector3DReadOnly finalCoMVelocity)
   {
      this.startTimeInPhase = startTimeInPhase;
      this.midpointTimeInPhase = midpointTimeInPhase;
      this.finalTimeInPhase = finalTimeInPhase;

      this.omega = omega;
      this.gravityZ = gravityZ;

      this.firstContactState = firstContactState;
      this.secondContactState = secondContactState;

      double firstSegmentDuration = this.midpointTimeInPhase - this.startTimeInPhase;
      double secondSegmentDuration = this.midpointTimeInPhase - this.startTimeInPhase;

      double initialPositionConstant0 = CoMTrajectoryPlannerTools.getFirstCoefficientCoMPositionMultiplier(firstContactState, 0.0, omega);
      double initialPositionConstant1 = CoMTrajectoryPlannerTools.getSecondCoefficientCoMPositionMultiplier(firstContactState, 0.0, omega);
      double initialPositionConstant2 = getThirdCoefficientCoMPositionMultiplier(firstContactState, 0.0, gravityZ);
      double initialPositionConstant3 = getFourthCoefficientCoMPositionMultiplier(firstContactState, 0.0, gravityZ);

      double midpointFirstPositionConstant0 = CoMTrajectoryPlannerTools
            .getFirstCoefficientCoMPositionMultiplier(firstContactState, firstSegmentDuration, omega);
      double midpointFirstPositionConstant1 = CoMTrajectoryPlannerTools
            .getSecondCoefficientCoMPositionMultiplier(firstContactState, firstSegmentDuration, omega);
      double midpointFirstPositionConstant2 = getThirdCoefficientCoMPositionMultiplier(firstContactState, firstSegmentDuration, gravityZ);
      double midpointFirstPositionConstant3 = getFourthCoefficientCoMPositionMultiplier(firstContactState, firstSegmentDuration, gravityZ);

      double midpointSecondPositionConstant0 = CoMTrajectoryPlannerTools.getFirstCoefficientCoMPositionMultiplier(secondContactState, 0.0, omega);
      double midpointSecondPositionConstant1 = CoMTrajectoryPlannerTools.getSecondCoefficientCoMPositionMultiplier(secondContactState, 0.0, omega);
      double midpointSecondPositionConstant2 = getThirdCoefficientCoMPositionMultiplier(secondContactState, 0.0, gravityZ);
      double midpointSecondPositionConstant3 = getFourthCoefficientCoMPositionMultiplier(secondContactState, 0.0, gravityZ);

      double finalPositionConstant0 = CoMTrajectoryPlannerTools.getFirstCoefficientCoMPositionMultiplier(secondContactState, secondSegmentDuration, omega);
      double finalPositionConstant1 = CoMTrajectoryPlannerTools.getSecondCoefficientCoMPositionMultiplier(secondContactState, secondSegmentDuration, omega);
      double finalPositionConstant2 = getThirdCoefficientCoMPositionMultiplier(secondContactState, secondSegmentDuration, gravityZ);
      double finalPositionConstant3 = getFourthCoefficientCoMPositionMultiplier(secondContactState, secondSegmentDuration, gravityZ);

      double initialVelocityConstant0 = CoMTrajectoryPlannerTools.getFirstCoefficientCoMVelocityMultiplier(firstContactState, 0.0, omega);
      double initialVelocityConstant1 = CoMTrajectoryPlannerTools.getSecondCoefficientCoMVelocityMultiplier(firstContactState, 0.0, omega);
      double initialVelocityConstant2 = getThirdCoefficientCoMVelocityMultiplier(secondContactState, secondSegmentDuration, gravityZ);
      double initialVelocityConstant3 = getFourthCoefficientCoMVelocityMultiplier(secondContactState, secondSegmentDuration, gravityZ);

      double midpointFirstVelocityConstant0 = CoMTrajectoryPlannerTools
            .getFirstCoefficientCoMVelocityMultiplier(firstContactState, firstSegmentDuration, omega);
      double midpointFirstVelocityConstant1 = CoMTrajectoryPlannerTools
            .getSecondCoefficientCoMVelocityMultiplier(firstContactState, firstSegmentDuration, omega);
      double midpointFirstVelocityConstant2 = getThirdCoefficientCoMVelocityMultiplier(firstContactState, firstSegmentDuration, gravityZ);
      double midpointFirstVelocityConstant3 = getFourthCoefficientCoMVelocityMultiplier(firstContactState, firstSegmentDuration, gravityZ);

      double midpointSecondVelocityConstant0 = CoMTrajectoryPlannerTools.getFirstCoefficientCoMVelocityMultiplier(secondContactState, 0.0, omega);
      double midpointSecondVelocityConstant1 = CoMTrajectoryPlannerTools.getSecondCoefficientCoMVelocityMultiplier(secondContactState, 0.0, omega);
      double midpointSecondVelocityConstant2 = getThirdCoefficientCoMVelocityMultiplier(secondContactState, 0.0, gravityZ);
      double midpointSecondVelocityConstant3 = getFourthCoefficientCoMVelocityMultiplier(secondContactState, 0.0, gravityZ);

      double finalVelocityConstant0 = CoMTrajectoryPlannerTools.getFirstCoefficientCoMVelocityMultiplier(secondContactState, secondSegmentDuration, omega);
      double finalVelocityConstant1 = CoMTrajectoryPlannerTools.getSecondCoefficientCoMVelocityMultiplier(secondContactState, secondSegmentDuration, omega);
      double finalVelocityConstant2 = getThirdCoefficientCoMVelocityMultiplier(secondContactState, secondSegmentDuration, gravityZ);
      double finalVelocityConstant3 = getFourthCoefficientCoMVelocityMultiplier(secondContactState, secondSegmentDuration, gravityZ);

      coefficientMultipliers.zero();
      coefficientMultipliersInv.zero();
      xCoefficientConstants.zero();
      yCoefficientConstants.zero();
      zCoefficientConstants.zero();
      xCoefficientVector.zero();
      yCoefficientVector.zero();
      zCoefficientVector.zero();

      int constraintRow = 0;
      // set initial position continuity
      coefficientMultipliers.set(constraintRow, 0, initialPositionConstant0);
      coefficientMultipliers.set(constraintRow, 1, initialPositionConstant1);
      coefficientMultipliers.set(constraintRow, 2, initialPositionConstant2);
      coefficientMultipliers.set(constraintRow, 3, initialPositionConstant3);
      xCoefficientConstants.set(constraintRow, 0, initialCoMPosition.getX());
      yCoefficientConstants.set(constraintRow, 0, initialCoMPosition.getY());
      zCoefficientConstants.set(constraintRow, 0, initialCoMPosition.getZ());

      constraintRow++;

      // set initial velocity continuity
      coefficientMultipliers.set(constraintRow, 0, initialVelocityConstant0);
      coefficientMultipliers.set(constraintRow, 1, initialVelocityConstant1);
      coefficientMultipliers.set(constraintRow, 2, initialVelocityConstant2);
      coefficientMultipliers.set(constraintRow, 3, initialVelocityConstant3);
      xCoefficientConstants.set(constraintRow, 0, initialCoMVelocity.getX());
      yCoefficientConstants.set(constraintRow, 0, initialCoMVelocity.getY());
      zCoefficientConstants.set(constraintRow, 0, initialCoMVelocity.getZ());

      constraintRow++;

      // set midpoint position continuity
      coefficientMultipliers.set(constraintRow, 0, midpointFirstPositionConstant0);
      coefficientMultipliers.set(constraintRow, 1, midpointFirstPositionConstant1);
      coefficientMultipliers.set(constraintRow, 2, midpointFirstPositionConstant2);
      coefficientMultipliers.set(constraintRow, 3, midpointFirstPositionConstant3);
      coefficientMultipliers.set(constraintRow, 4, -midpointSecondPositionConstant0);
      coefficientMultipliers.set(constraintRow, 5, -midpointSecondPositionConstant1);
      coefficientMultipliers.set(constraintRow, 6, -midpointSecondPositionConstant2);
      coefficientMultipliers.set(constraintRow, 7, -midpointSecondPositionConstant3);
      xCoefficientConstants.set(constraintRow, 0, 0.0);
      yCoefficientConstants.set(constraintRow, 0, 0.0);
      zCoefficientConstants.set(constraintRow, 0, 0.0);

      constraintRow++;

      // set midpoint velocity continuity
      coefficientMultipliers.set(constraintRow, 0, midpointFirstVelocityConstant0);
      coefficientMultipliers.set(constraintRow, 1, midpointFirstVelocityConstant1);
      coefficientMultipliers.set(constraintRow, 2, midpointFirstVelocityConstant2);
      coefficientMultipliers.set(constraintRow, 3, midpointFirstVelocityConstant3);
      coefficientMultipliers.set(constraintRow, 4, -midpointSecondVelocityConstant0);
      coefficientMultipliers.set(constraintRow, 5, -midpointSecondVelocityConstant1);
      coefficientMultipliers.set(constraintRow, 6, -midpointSecondVelocityConstant2);
      coefficientMultipliers.set(constraintRow, 7, -midpointSecondVelocityConstant3);
      xCoefficientConstants.set(constraintRow, 0, 0.0);
      yCoefficientConstants.set(constraintRow, 0, 0.0);
      zCoefficientConstants.set(constraintRow, 0, 0.0);

      constraintRow++;

      // set final position continuity
      coefficientMultipliers.set(constraintRow, 4, finalPositionConstant0);
      coefficientMultipliers.set(constraintRow, 5, finalPositionConstant1);
      coefficientMultipliers.set(constraintRow, 6, finalPositionConstant2);
      coefficientMultipliers.set(constraintRow, 7, finalPositionConstant3);
      xCoefficientConstants.set(constraintRow, 0, finalCoMPosition.getX());
      yCoefficientConstants.set(constraintRow, 0, finalCoMPosition.getY());
      zCoefficientConstants.set(constraintRow, 0, finalCoMPosition.getZ());

      constraintRow++;

      // set initial velocity continuity
      coefficientMultipliers.set(constraintRow, 4, finalVelocityConstant0);
      coefficientMultipliers.set(constraintRow, 5, finalVelocityConstant1);
      coefficientMultipliers.set(constraintRow, 6, finalVelocityConstant2);
      coefficientMultipliers.set(constraintRow, 7, finalVelocityConstant3);
      xCoefficientConstants.set(constraintRow, 0, finalCoMVelocity.getX());
      yCoefficientConstants.set(constraintRow, 0, finalCoMVelocity.getY());
      zCoefficientConstants.set(constraintRow, 0, finalCoMVelocity.getZ());

      solver.setA(coefficientMultipliers);
      solver.invert(coefficientMultipliersInv);

      solver.solve(xCoefficientConstants, xCoefficientVector);
      solver.solve(yCoefficientConstants, yCoefficientVector);
      solver.solve(zCoefficientConstants, zCoefficientVector);
   }

   private final FramePoint3D firstCoefficient = new FramePoint3D();
   private final FramePoint3D secondCoefficient = new FramePoint3D();
   private final FramePoint3D thirdCoefficient = new FramePoint3D();
   private final FramePoint3D fourthCoefficient = new FramePoint3D();

   public void compute(double timeInPhase)
   {
      ContactState contactState;
      double timeInSegment;
      if (timeInPhase > midpointTimeInPhase)
      { // in second segment
         timeInSegment = timeInPhase - midpointTimeInPhase;
         contactState = secondContactState;

         firstCoefficient.setX(xCoefficientConstants.get(4));
         firstCoefficient.setY(yCoefficientConstants.get(4));
         firstCoefficient.setZ(zCoefficientConstants.get(4));

         secondCoefficient.setX(xCoefficientConstants.get(5));
         secondCoefficient.setY(yCoefficientConstants.get(5));
         secondCoefficient.setZ(zCoefficientConstants.get(5));

         thirdCoefficient.setX(xCoefficientConstants.get(6));
         thirdCoefficient.setY(yCoefficientConstants.get(6));
         thirdCoefficient.setZ(zCoefficientConstants.get(6));

         fourthCoefficient.setX(xCoefficientConstants.get(7));
         fourthCoefficient.setY(yCoefficientConstants.get(7));
         fourthCoefficient.setZ(zCoefficientConstants.get(7));
      }
      else
      {
         timeInSegment = timeInPhase - startTimeInPhase;
         contactState = firstContactState;

         firstCoefficient.setX(xCoefficientConstants.get(0));
         firstCoefficient.setY(yCoefficientConstants.get(0));
         firstCoefficient.setZ(zCoefficientConstants.get(0));

         secondCoefficient.setX(xCoefficientConstants.get(1));
         secondCoefficient.setY(yCoefficientConstants.get(1));
         secondCoefficient.setZ(zCoefficientConstants.get(1));

         thirdCoefficient.setX(xCoefficientConstants.get(2));
         thirdCoefficient.setY(yCoefficientConstants.get(2));
         thirdCoefficient.setZ(zCoefficientConstants.get(2));

         fourthCoefficient.setX(xCoefficientConstants.get(3));
         fourthCoefficient.setY(yCoefficientConstants.get(3));
         fourthCoefficient.setZ(zCoefficientConstants.get(3));
      }


      // FIXME
      double firstPositionMultiplier = CoMTrajectoryPlannerTools.getFirstCoefficientCoMPositionMultiplier(contactState, timeInSegment, omega);
      double secondPositionMultiplier = CoMTrajectoryPlannerTools.getSecondCoefficientCoMPositionMultiplier(contactState, timeInSegment, omega);
      double thirdPositionMultiplier = getThirdCoefficientCoMPositionMultiplier(contactState, timeInSegment, gravityZ);

      double firstVelocityMultiplier = CoMTrajectoryPlannerTools.getFirstCoefficientCoMVelocityMultiplier(contactState, timeInSegment, omega);
      double secondVelocityMultiplier = CoMTrajectoryPlannerTools.getSecondCoefficientCoMVelocityMultiplier(contactState, timeInSegment, omega);
      double thirdVelocityMultiplier = getThirdCoefficientCoMVelocityMultiplier(contactState, timeInSegment, gravityZ);

      double firstAccelerationMultiplier = CoMTrajectoryPlannerTools.getFirstCoefficientCoMAccelerationMultiplier(contactState, timeInSegment, omega);
      double secondAccelerationMultiplier = CoMTrajectoryPlannerTools.getSecondCoefficientCoMAccelerationMultiplier(contactState, timeInSegment, omega);
      double thirdAccelerationMultiplier = CoMTrajectoryPlannerTools.getGravityAccelerationEffect(contactState, gravityZ);

      desiredComPosition.setToZero();
      desiredComPosition.scaleAdd(firstPositionMultiplier, firstCoefficient, desiredComPosition);
      desiredComPosition.scaleAdd(secondPositionMultiplier, secondCoefficient, desiredComPosition);
      desiredComPosition.scaleAdd(thirdPositionMultiplier, thirdCoefficient, desiredComPosition);
//     desiredComPosition.scaleAdd(fourthPositionMultiplier, fourthCoefficient, desiredComPosition); // FIXME

      desiredComVelocity.setToZero();
      desiredComVelocity.scaleAdd(firstVelocityMultiplier, firstCoefficient, desiredComVelocity);
      desiredComVelocity.scaleAdd(secondVelocityMultiplier, secondCoefficient, desiredComVelocity);
      desiredComVelocity.scaleAdd(thirdVelocityMultiplier, thirdCoefficient, desiredComVelocity);

      desiredComAcceleration.setToZero();
      desiredComAcceleration.scaleAdd(firstAccelerationMultiplier, firstCoefficient, desiredComAcceleration);
      desiredComAcceleration.scaleAdd(secondAccelerationMultiplier, secondCoefficient, desiredComAcceleration);
      desiredComAcceleration.scaleAdd(thirdAccelerationMultiplier, thirdCoefficient, desiredComAcceleration);
   }

   public FramePoint3DReadOnly getDesiredComPosition()
   {
      return desiredComPosition;
   }

   public FrameVector3DReadOnly getDesiredComVelocity()
   {
      return desiredComVelocity;
   }

   public FrameVector3DReadOnly getDesiredComAcceleration()
   {
      return desiredComAcceleration;
   }

   private static double getThirdCoefficientCoMPositionMultiplier(ContactState contactState, double time, double gravityZ)
   {
      return contactState == ContactState.IN_CONTACT ? time : CoMTrajectoryPlannerTools.getGravityPositionEffect(contactState, time, gravityZ);
   }

   private static double getFourthCoefficientCoMPositionMultiplier(ContactState contactState, double time, double gravityZ)
   {
      return contactState == ContactState.IN_CONTACT ? 1.0 : 0.0;
   }

   private static double getThirdCoefficientCoMVelocityMultiplier(ContactState contactState, double time, double gravityZ)
   {
      return contactState == ContactState.IN_CONTACT ? 1.0 : CoMTrajectoryPlannerTools.getGravityVelocityEffect(contactState, time, gravityZ);
   }

   private static double getFourthCoefficientCoMVelocityMultiplier(ContactState contactState, double time, double gravityZ)
   {
      return 0.0;
   }
}
