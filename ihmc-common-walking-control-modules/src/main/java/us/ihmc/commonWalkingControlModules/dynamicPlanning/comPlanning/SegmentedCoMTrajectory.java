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

   private final DenseMatrix64F coefficientMultipliers = new DenseMatrix64F(6, 6);
   private final DenseMatrix64F coefficientMultipliersInv = new DenseMatrix64F(6, 6);
   private final DenseMatrix64F xCoefficientConstants = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F yCoefficientConstants = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F zCoefficientConstants = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F xCoefficientVector = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F yCoefficientVector = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F zCoefficientVector = new DenseMatrix64F(6, 1);

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

      double initialPositionConstant0 = CoMTrajectoryPlannerTools.getFirstCoefficientPositionMultiplier(firstContactState, 0.0, omega);
      double initialPositionConstant1 = CoMTrajectoryPlannerTools.getSecondCoefficientPositionMultiplier(firstContactState, 0.0, omega);
      double initialPositionConstant2 = getThirdCoefficientPositionMultiplier(firstContactState, 0.0, gravityZ);

      double midpointFirstPositionConstant0 = CoMTrajectoryPlannerTools.getFirstCoefficientPositionMultiplier(firstContactState, firstSegmentDuration, omega);
      double midpointFirstPositionConstant1 = CoMTrajectoryPlannerTools.getSecondCoefficientPositionMultiplier(firstContactState, firstSegmentDuration, omega);
      double midpointFirstPositionConstant2 = getThirdCoefficientPositionMultiplier(firstContactState, firstSegmentDuration, gravityZ);

      double midpointSecondPositionConstant0 = CoMTrajectoryPlannerTools.getFirstCoefficientPositionMultiplier(secondContactState, 0.0, omega);
      double midpointSecondPositionConstant1 = CoMTrajectoryPlannerTools.getSecondCoefficientPositionMultiplier(secondContactState, 0.0, omega);
      double midpointSecondPositionConstant2 = getThirdCoefficientPositionMultiplier(secondContactState, 0.0, gravityZ);

      double finalPositionConstant0 = CoMTrajectoryPlannerTools.getFirstCoefficientPositionMultiplier(secondContactState, secondSegmentDuration, omega);
      double finalPositionConstant1 = CoMTrajectoryPlannerTools.getSecondCoefficientPositionMultiplier(secondContactState, secondSegmentDuration, omega);
      double finalPositionConstant2 = getThirdCoefficientPositionMultiplier(secondContactState, secondSegmentDuration, gravityZ);

      double initialVelocityConstant0 = CoMTrajectoryPlannerTools.getFirstCoefficientVelocityMultiplier(firstContactState, 0.0, omega);
      double initialVelocityConstant1 = CoMTrajectoryPlannerTools.getSecondCoefficientVelocityMultiplier(firstContactState, 0.0, omega);
      double initialVelocityConstant2 = CoMTrajectoryPlannerTools.getGravityVelocityEffect(firstContactState, 0.0, gravityZ);

      double midpointFirstVelocityConstant0 = CoMTrajectoryPlannerTools.getFirstCoefficientVelocityMultiplier(firstContactState, firstSegmentDuration, omega);
      double midpointFirstVelocityConstant1 = CoMTrajectoryPlannerTools.getSecondCoefficientVelocityMultiplier(firstContactState, firstSegmentDuration, omega);
      double midpointFirstVelocityConstant2 = CoMTrajectoryPlannerTools.getGravityVelocityEffect(firstContactState, firstSegmentDuration, gravityZ);

      double midpointSecondVelocityConstant0 = CoMTrajectoryPlannerTools.getFirstCoefficientVelocityMultiplier(secondContactState, 0.0, omega);
      double midpointSecondVelocityConstant1 = CoMTrajectoryPlannerTools.getSecondCoefficientVelocityMultiplier(secondContactState, 0.0, omega);
      double midpointSecondVelocityConstant2 = CoMTrajectoryPlannerTools.getGravityVelocityEffect(secondContactState, 0.0, gravityZ);

      double finalVelocityConstant0 = CoMTrajectoryPlannerTools.getFirstCoefficientVelocityMultiplier(secondContactState, secondSegmentDuration, omega);
      double finalVelocityConstant1 = CoMTrajectoryPlannerTools.getSecondCoefficientVelocityMultiplier(secondContactState, secondSegmentDuration, omega);
      double finalVelocityConstant2 = CoMTrajectoryPlannerTools.getGravityVelocityEffect(secondContactState, secondSegmentDuration, gravityZ);

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
      xCoefficientConstants.set(constraintRow, 0, initialCoMPosition.getX());
      yCoefficientConstants.set(constraintRow, 0, initialCoMPosition.getY());
      zCoefficientConstants.set(constraintRow, 0, initialCoMPosition.getZ());

      constraintRow++;

      // set initial velocity continuity
      coefficientMultipliers.set(constraintRow, 0, initialVelocityConstant0);
      coefficientMultipliers.set(constraintRow, 1, initialVelocityConstant1);
      coefficientMultipliers.set(constraintRow, 2, initialVelocityConstant2);
      xCoefficientConstants.set(constraintRow, 0, initialCoMVelocity.getX());
      yCoefficientConstants.set(constraintRow, 0, initialCoMVelocity.getY());
      zCoefficientConstants.set(constraintRow, 0, initialCoMVelocity.getZ());

      constraintRow++;

      // set midpoint position continuity
      coefficientMultipliers.set(constraintRow, 0, midpointFirstPositionConstant0);
      coefficientMultipliers.set(constraintRow, 1, midpointFirstPositionConstant1);
      coefficientMultipliers.set(constraintRow, 2, midpointFirstPositionConstant2);
      coefficientMultipliers.set(constraintRow, 3, -midpointSecondPositionConstant0);
      coefficientMultipliers.set(constraintRow, 4, -midpointSecondPositionConstant1);
      coefficientMultipliers.set(constraintRow, 5, -midpointSecondPositionConstant2);
      xCoefficientConstants.set(constraintRow, 0, 0.0);
      yCoefficientConstants.set(constraintRow, 0, 0.0);
      zCoefficientConstants.set(constraintRow, 0, 0.0);

      constraintRow++;

      // set midpoint velocity continuity
      coefficientMultipliers.set(constraintRow, 0, midpointFirstVelocityConstant0);
      coefficientMultipliers.set(constraintRow, 1, midpointFirstVelocityConstant1);
      coefficientMultipliers.set(constraintRow, 2, midpointFirstVelocityConstant2);
      coefficientMultipliers.set(constraintRow, 3, -midpointSecondVelocityConstant0);
      coefficientMultipliers.set(constraintRow, 4, -midpointSecondVelocityConstant1);
      coefficientMultipliers.set(constraintRow, 5, -midpointSecondVelocityConstant2);
      xCoefficientConstants.set(constraintRow, 0, 0.0);
      yCoefficientConstants.set(constraintRow, 0, 0.0);
      zCoefficientConstants.set(constraintRow, 0, 0.0);

      constraintRow++;

      // set final position continuity
      coefficientMultipliers.set(constraintRow, 3, finalPositionConstant0);
      coefficientMultipliers.set(constraintRow, 4, finalPositionConstant1);
      coefficientMultipliers.set(constraintRow, 5, finalPositionConstant2);
      xCoefficientConstants.set(constraintRow, 0, finalCoMPosition.getX());
      yCoefficientConstants.set(constraintRow, 0, finalCoMPosition.getY());
      zCoefficientConstants.set(constraintRow, 0, finalCoMPosition.getZ());

      constraintRow++;

      // set initial velocity continuity
      coefficientMultipliers.set(constraintRow, 3, finalVelocityConstant0);
      coefficientMultipliers.set(constraintRow, 4, finalVelocityConstant1);
      coefficientMultipliers.set(constraintRow, 5, finalVelocityConstant2);
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

   public void compute(double timeInPhase)
   {
      ContactState contactState;
      double timeInSegment;
      if (timeInPhase > midpointTimeInPhase)
      { // in second segment
         timeInSegment = timeInPhase - midpointTimeInPhase;
         contactState = secondContactState;

         firstCoefficient.setX(xCoefficientConstants.get(3));
         firstCoefficient.setY(yCoefficientConstants.get(3));
         firstCoefficient.setZ(zCoefficientConstants.get(3));

         secondCoefficient.setX(xCoefficientConstants.get(4));
         secondCoefficient.setY(yCoefficientConstants.get(4));
         secondCoefficient.setZ(zCoefficientConstants.get(4));

         thirdCoefficient.setX(xCoefficientConstants.get(5));
         thirdCoefficient.setY(yCoefficientConstants.get(5));
         thirdCoefficient.setZ(zCoefficientConstants.get(5));
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
      }

      double firstPositionMultiplier = CoMTrajectoryPlannerTools.getFirstCoefficientPositionMultiplier(contactState, timeInSegment, omega);
      double secondPositionMultiplier = CoMTrajectoryPlannerTools.getSecondCoefficientPositionMultiplier(contactState, timeInSegment, omega);
      double thirdPositionMultiplier = getThirdCoefficientPositionMultiplier(contactState, timeInSegment, gravityZ);

      double firstVelocityMultiplier = CoMTrajectoryPlannerTools.getFirstCoefficientVelocityMultiplier(contactState, timeInSegment, omega);
      double secondVelocityMultiplier = CoMTrajectoryPlannerTools.getSecondCoefficientVelocityMultiplier(contactState, timeInSegment, omega);
      double thirdVelocityMultiplier = CoMTrajectoryPlannerTools.getGravityVelocityEffect(contactState, timeInSegment, gravityZ);

      double firstAccelerationMultiplier = CoMTrajectoryPlannerTools.getFirstCoefficientAccelerationMultiplier(contactState, timeInSegment, omega);
      double secondAccelerationMultiplier = CoMTrajectoryPlannerTools.getSecondCoefficientAccelerationMultiplier(contactState, timeInSegment, omega);
      double thirdAccelerationMultiplier = CoMTrajectoryPlannerTools.getGravityAccelerationEffect(contactState, gravityZ);

      desiredComPosition.setToZero();
      desiredComPosition.scaleAdd(firstPositionMultiplier, firstCoefficient, desiredComPosition);
      desiredComPosition.scaleAdd(secondPositionMultiplier, secondCoefficient, desiredComPosition);
      desiredComPosition.scaleAdd(thirdPositionMultiplier, thirdCoefficient, desiredComPosition);

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

   private static double getThirdCoefficientPositionMultiplier(ContactState contactState, double time, double gravityZ)
   {
      return contactState == ContactState.IN_CONTACT ? 1.0 : CoMTrajectoryPlannerTools.getGravityPositionEffect(contactState, time, gravityZ);
   }

}
