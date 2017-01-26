package us.ihmc.quadrupedRobotics.optimization.modelPredictiveControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.convexOptimization.quadraticProgram.ConstrainedQPSolver;
import us.ihmc.convexOptimization.quadraticProgram.QuadProgSolver;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.DivergentComponentOfMotionEstimator;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.LinearInvertedPendulumModel;
import us.ihmc.quadrupedRobotics.planning.*;
import us.ihmc.quadrupedRobotics.planning.trajectory.QuadrupedPiecewiseConstantCopTrajectory;
import us.ihmc.quadrupedRobotics.util.PreallocatedList;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class QuadrupedDcmBasedMpcOptimizationWithLaneChange implements QuadrupedMpcOptimizationWithLaneChange
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FramePoint currentDcmEstimate;
   private final DivergentComponentOfMotionEstimator dcmPositionEstimator;
   private final LinearInvertedPendulumModel linearInvertedPendulumModel;
   private final QuadrupedTimedContactSequence timedContactSequence;
   private final QuadrupedPiecewiseConstantCopTrajectory piecewiseConstantCopTrajectory;

   private final ConstrainedQPSolver qpSolver = new QuadProgSolver();
   private final DenseMatrix64F qpSolutionVector = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F qpCostVector = new DenseMatrix64F(100, 1);
   private final DenseMatrix64F qpCostMatrix = new DenseMatrix64F(100, 100);
   private final DenseMatrix64F qpEqualityVector = new DenseMatrix64F(100, 1);
   private final DenseMatrix64F qpEqualityMatrix = new DenseMatrix64F(100, 100);
   private final DenseMatrix64F qpInequalityVector = new DenseMatrix64F(100, 1);
   private final DenseMatrix64F qpInequalityMatrix = new DenseMatrix64F(100, 100);

   private final DenseMatrix64F x0 = new DenseMatrix64F(100, 1);
   private final DenseMatrix64F y0 = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F B = new DenseMatrix64F(100, 6);
   private final DenseMatrix64F C = new DenseMatrix64F(2, 100);
   private final DenseMatrix64F S = new DenseMatrix64F(2, 100);
   private final DenseMatrix64F CmS = new DenseMatrix64F(2, 100);
   private final DenseMatrix64F CmSB = new DenseMatrix64F(2, 6);
   private final DenseMatrix64F CmSx0py0 = new DenseMatrix64F(2, 1);

   private int numberOfContacts = 0;
   private int numberOfIntervals = 0;
   private int numberOfPreviewSteps = 0;

   private YoFramePoint yoCmpPositionSetpoint = new YoFramePoint("cmpPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   private YoFrameVector yoStepAdjustmentVector = new YoFrameVector("stepAdjustmentVector", ReferenceFrame.getWorldFrame(), registry);

   public QuadrupedDcmBasedMpcOptimizationWithLaneChange(DivergentComponentOfMotionEstimator dcmPositionEstimator, int maxPreviewSteps,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.linearInvertedPendulumModel = dcmPositionEstimator.getLinearInvertedPendulumModel();
      this.dcmPositionEstimator = dcmPositionEstimator;
      this.currentDcmEstimate = new FramePoint();
      this.timedContactSequence = new QuadrupedTimedContactSequence(0, 2 * maxPreviewSteps + 4);
      this.piecewiseConstantCopTrajectory = new QuadrupedPiecewiseConstantCopTrajectory(timedContactSequence.capacity());

      if (graphicsListRegistry != null)
      {
         String cmpPositionGraphicName = registry.getName() + "cmpPositionSetpoint";
         YoGraphicPosition cmpPositionGraphic = new YoGraphicPosition(cmpPositionGraphicName, yoCmpPositionSetpoint, 0.025, YoAppearance.Chartreuse());
         graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), cmpPositionGraphic);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), cmpPositionGraphic.createArtifact());
      }
      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
      timedContactSequence.initialize();
   }

   @Override
   public void compute(FrameVector stepAdjustmentVector, FramePoint cmpPositionSetpoint, PreallocatedList<QuadrupedTimedStep> queuedSteps,
         QuadrantDependentList<FramePoint> currentSolePosition, QuadrantDependentList<ContactState> currentContactState, FramePoint currentComPosition,
         FrameVector currentComVelocity, double currentTime, QuadrupedMpcOptimizationWithLaneChangeSettings settings)
   {
      // Compute step adjustment and contact pressure by solving the following QP:
      // min_u u'Au
      // s.t
      // (C - S)Bu + (C - S)x0 + y0 = 0
      // u0 + u1 + u2 + u3 - 1 = 0
      // u0 >= 0
      // u1 >=0
      // u2 >= 0
      // u3 >= 0
      // where u = [u0, u1, u2, u3, u4, u5]',
      // u0, u1, u2, u3 are the normalized contact pressures for each quadrant and
      // u4, u5 are the x and y step adjustment in meters

      // Compute current divergent component of motion.
      dcmPositionEstimator.compute(currentDcmEstimate, currentComVelocity);
      currentDcmEstimate.changeFrame(ReferenceFrame.getWorldFrame());
      cmpPositionSetpoint.changeFrame(ReferenceFrame.getWorldFrame());
      stepAdjustmentVector.changeFrame(ReferenceFrame.getWorldFrame());

      // Compute current number of contacts.
      numberOfContacts = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (currentContactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            numberOfContacts++;
         }
      }

      // Compute number of steps occurring inside preview window.
      numberOfPreviewSteps = 1;
      for (int i = 1; i < queuedSteps.size(); i++)
      {
         QuadrupedTimedStep step = queuedSteps.get(i);
         if (step.getTimeInterval().getEndTime() - currentTime < settings.getMaximumPreviewTime())
         {
            numberOfPreviewSteps++;
         }
      }

      // Compute nominal piecewise center of pressure plan.
      timedContactSequence.update(queuedSteps, currentSolePosition, currentContactState, currentTime);
      piecewiseConstantCopTrajectory.initializeTrajectory(timedContactSequence);
      numberOfIntervals = piecewiseConstantCopTrajectory.getNumberOfIntervals();

      // Solve constrained quadratic program.
      DenseMatrix64F A = qpCostMatrix;
      DenseMatrix64F b = qpCostVector;
      DenseMatrix64F Aeq = qpEqualityMatrix;
      DenseMatrix64F beq = qpEqualityVector;
      DenseMatrix64F Ain = qpInequalityMatrix;
      DenseMatrix64F bin = qpInequalityVector;

      initializeCostTerms(currentContactState, settings);
      initializeEqualityConstraints(currentContactState, currentSolePosition);
      initializeInequalityConstraints(settings);

      DenseMatrix64F u = qpSolutionVector;
      u.reshape(numberOfContacts + 2, 1);
      try
      {
         qpSolver.solve(A, b, Aeq, beq, Ain, bin, u, false);
      }
      catch (NoConvergenceException e)
      {
         System.err.println("NoConvergenceException: " + e.getMessage());
      }

      // Compute optimal centroidal moment pivot and step adjustment
      int rowOffset = 0;
      cmpPositionSetpoint.setToZero();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (currentContactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            double normalizedContactPressure = u.get(rowOffset++, 0);
            currentSolePosition.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
            addPointWithScaleFactor(cmpPositionSetpoint, currentSolePosition.get(robotQuadrant), normalizedContactPressure);
         }
      }
      for (Direction direction : Direction.values2D())
      {
         stepAdjustmentVector.set(direction, u.get(rowOffset++, 0));
      }

      // Update logging variables
      yoCmpPositionSetpoint.setAndMatchFrame(cmpPositionSetpoint);
      yoStepAdjustmentVector.setAndMatchFrame(stepAdjustmentVector);
   }

   private void initializeCostTerms(QuadrantDependentList<ContactState> currentContactState, QuadrupedMpcOptimizationWithLaneChangeSettings settings)
   {
      // Initialize cost terms. (min_u u'Au + b'u)
      DenseMatrix64F A = qpCostMatrix;
      A.reshape(numberOfContacts + 2, numberOfContacts + 2);
      A.zero();
      for (int i = 0; i < numberOfContacts; i++)
      {
         A.set(i, i, settings.getCopAdjustmentCost());
      }
      for (int i = numberOfContacts; i < numberOfContacts + 2; i++)
      {
         A.set(i, i, settings.getStepAdjustmentCost());
      }

      DenseMatrix64F b = qpCostVector;
      b.reshape(numberOfContacts + 2, 1);
      b.zero();

      int rowOffset = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (currentContactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            b.set(rowOffset++, 0, piecewiseConstantCopTrajectory.getNormalizedPressureAtStartOfInterval(0).get(robotQuadrant).doubleValue());
         }
      }
      CommonOps.multTransA(A, b, b);
      CommonOps.scale(-2, b, b);
   }

   private void initializeEqualityConstraints(QuadrantDependentList<ContactState> currentContactState, QuadrantDependentList<FramePoint> currentSolePosition)
   {
      // Initialize equality constraints. (Aeq u = beq)
      x0.reshape(2 * numberOfIntervals, 1);                    // center of pressure offset
      y0.reshape(2, 1);                                        // final divergent component of motion offset
      B.reshape(2 * numberOfIntervals, numberOfContacts + 2);  // center of pressure map
      C.reshape(2, 2 * numberOfIntervals);                     // final divergent component of motion map
      S.reshape(2, 2 * numberOfIntervals);                     // final interval selection matrix
      B.zero();
      C.zero();
      S.zero();

      int rowOffset = 0;
      int columnOffset = 0;
      for (Direction direction : Direction.values2D())
      {
         columnOffset = 0;
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (currentContactState.get(robotQuadrant) == ContactState.IN_CONTACT)
            {
               currentSolePosition.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
               B.set(rowOffset, columnOffset, currentSolePosition.get(robotQuadrant).get(direction));
               columnOffset++;
            }
         }

         for (int i = 0; i < numberOfIntervals; i++)
         {
            piecewiseConstantCopTrajectory.getCopPositionAtStartOfInterval(i).changeFrame(ReferenceFrame.getWorldFrame());
            x0.set(i * 2 + rowOffset, 0, piecewiseConstantCopTrajectory.getCopPositionAtStartOfInterval(i).get(direction));
            B.set(i * 2 + rowOffset, numberOfContacts + rowOffset, piecewiseConstantCopTrajectory.getNormalizedPressureContributedByQueuedSteps(i));
         }
         x0.set(rowOffset, 0, 0);

         double naturalFrequency = linearInvertedPendulumModel.getNaturalFrequency();
         for (int i = numberOfIntervals - 2; i >= 0; i--)
         {
            double tn = piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(numberOfIntervals - 1) - piecewiseConstantCopTrajectory
                  .getTimeAtStartOfInterval(i + 1);
            double ti = piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(i + 1) - piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(i);
            double expn = Math.exp(naturalFrequency * tn);
            double expi = Math.exp(naturalFrequency * ti);
            C.set(rowOffset, i * 2 + rowOffset, expn * (1 - expi));
         }
         C.set(rowOffset, 2 * numberOfIntervals - 2 + rowOffset, 0);
         S.set(rowOffset, 2 * numberOfIntervals - 2 + rowOffset, 1);

         double previewTime =
               piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(numberOfIntervals - 1) - piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(0);
         y0.set(rowOffset, 0, Math.exp(naturalFrequency * previewTime) * currentDcmEstimate.get(direction));
         rowOffset++;
      }

      CmS.reshape(2, 2 * numberOfIntervals);
      CmSB.reshape(2, numberOfContacts + 2);
      CmSx0py0.reshape(2, 1);
      CommonOps.subtract(C, S, CmS);
      CommonOps.mult(CmS, B, CmSB);
      CmSx0py0.set(y0);
      CommonOps.multAdd(CmS, x0, CmSx0py0);

      DenseMatrix64F Aeq = qpEqualityMatrix;
      Aeq.reshape(3, numberOfContacts + 2);
      Aeq.zero();
      for (int i = 0; i < numberOfContacts + 2; i++)
      {
         Aeq.set(0, i, CmSB.get(0, i));
         Aeq.set(1, i, CmSB.get(1, i));
      }
      for (int i = 0; i < numberOfContacts; i++)
      {
         Aeq.set(2, i, 1);
      }

      DenseMatrix64F beq = qpEqualityVector;
      beq.reshape(3, 1);
      beq.zero();
      beq.set(0, 0, -CmSx0py0.get(0, 0));
      beq.set(1, 0, -CmSx0py0.get(1, 0));
      beq.set(2, 0, 1);

      for (int i = 0; i < 3; i++)
      {
         // Normalize constraint if beq > 1.
         if (Math.abs(beq.get(i, 0)) > 1.0)
         {
            for (int j = 0; j < Aeq.getNumCols(); j++)
            {
               Aeq.set(i, j, Aeq.get(i, j) / beq.get(i, 0));
            }
            beq.set(i, 0, 1.0);
         }
      }

   }

   private void initializeInequalityConstraints(QuadrupedMpcOptimizationWithLaneChangeSettings settings)
   {
      // Initialize inequality constraints. (Ain u <= bin)
      DenseMatrix64F Ain = qpInequalityMatrix;
      Ain.reshape(numberOfContacts, numberOfContacts + 2);
      Ain.zero();
      for (int i = 0; i < numberOfContacts; i++)
      {
         Ain.set(i, i, -1);
      }

      DenseMatrix64F bin = qpInequalityVector;
      bin.reshape(numberOfContacts, 1);
      bin.zero();
      for (int i = 0; i < numberOfContacts; i++)
      {
         bin.set(i, 0, -Math.min(Math.max(settings.getMinimumNormalizedContactPressure(), 0), 0.25));
      }
   }

   private void addPointWithScaleFactor(FramePoint point, FramePoint pointToAdd, double scaleFactor)
   {
      point.checkReferenceFrameMatch(pointToAdd);
      point.add(scaleFactor * pointToAdd.getX(), scaleFactor * pointToAdd.getY(), scaleFactor * pointToAdd.getZ());
   }
}
