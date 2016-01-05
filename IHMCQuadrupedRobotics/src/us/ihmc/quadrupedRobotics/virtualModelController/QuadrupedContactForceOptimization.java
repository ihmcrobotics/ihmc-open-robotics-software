package us.ihmc.quadrupedRobotics.virtualModelController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedContactForceOptimization
{
   private final ReferenceFrame comFrame;

   private final FrameVector comTorqueCommand;
   private final FrameVector comForceCommand;
   private final FrameVector comTorqueSolution;
   private final FrameVector comForceSolution;
   private final QuadrantDependentList<FrameVector> contactForceSolution;
   private final QuadrantDependentList<FramePoint> contactPosition;

   private final DenseMatrix64F comWrenchVector;
   private final DenseMatrix64F comWrenchMapMatrix;
   private final DenseMatrix64F comWrenchWeightMatrix;
   private final DenseMatrix64F contactForceVector;
   private final DenseMatrix64F contactForceRegularizationWeightMatrix;
   private final DenseMatrix64F constraintVector;
   private final DenseMatrix64F constraintMatrix;
   private final DenseMatrix64F temporaryMatrixA;
   private final DenseMatrix64F temporaryMatrixB;

   public QuadrupedContactForceOptimization(QuadrupedReferenceFrames referenceFrames)
   {
      comFrame = referenceFrames.getCenterOfMassZUpFrame();

      comTorqueCommand = new FrameVector(comFrame);
      comForceCommand = new FrameVector(comFrame);
      comTorqueSolution = new FrameVector(comFrame);
      comForceSolution = new FrameVector(comFrame);
      contactForceSolution = new QuadrantDependentList<>();
      contactPosition = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactForceSolution.set(robotQuadrant, new FrameVector(comFrame));
         contactPosition.set(robotQuadrant, new FramePoint(comFrame));
      }

      comWrenchVector = new DenseMatrix64F(6, 1);
      comWrenchMapMatrix = new DenseMatrix64F(6, 12);
      comWrenchWeightMatrix = new DenseMatrix64F(6, 6);
      contactForceVector = new DenseMatrix64F(12, 1);
      contactForceRegularizationWeightMatrix = new DenseMatrix64F(12, 12);
      constraintVector = new DenseMatrix64F(12, 1);
      constraintMatrix = new DenseMatrix64F(12, 12);
      temporaryMatrixA = new DenseMatrix64F(12, 12);
      temporaryMatrixB = new DenseMatrix64F(12, 12);
   }

   public void setComTorqueCommand(FrameVector comTorque)
   {
      comTorqueCommand.setIncludingFrame(comTorque);
   }

   public void setComForceCommand(FrameVector comForce)
   {
      comForceCommand.setIncludingFrame(comForce);
   }

   public void solve(QuadrantDependentList<FramePoint> solePosition, QuadrantDependentList<boolean[]> contactState,
         QuadrupedContactForceLimits contactForceLimits, QuadrupedContactForceOptimizationSettings optimizationSettings)
   {
      int numberOfContacts = getNumberOfContacts(contactState);
      initializeComWrenchMapMatrix(solePosition, contactState);
      initializeComWrenchVector(comTorqueCommand, comForceCommand);
      initializeContactForceVector(contactState);
      initializeContactForceOptimizationWeights(optimizationSettings, numberOfContacts);
      computeUnconstrainedContactForceSolution();
      limitContactForceSolution(contactForceLimits, contactState);
      computeComWrenchSolution();

      int rowOffset = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant)[0])
         {
            contactForceSolution.get(robotQuadrant).changeFrame(comFrame);
            contactForceSolution.get(robotQuadrant).setX(contactForceVector.get(0 + rowOffset, 0));
            contactForceSolution.get(robotQuadrant).setY(contactForceVector.get(1 + rowOffset, 0));
            contactForceSolution.get(robotQuadrant).setZ(contactForceVector.get(2 + rowOffset, 0));

            rowOffset += 3;
         }
         else
         {
            contactForceSolution.get(robotQuadrant).changeFrame(comFrame);
            contactForceSolution.get(robotQuadrant).setToZero();
         }
      }
      comTorqueSolution.changeFrame(comFrame);
      comTorqueSolution.setX(comWrenchVector.get(0, 0));
      comTorqueSolution.setY(comWrenchVector.get(1, 0));
      comTorqueSolution.setZ(comWrenchVector.get(2, 0));
      comForceSolution.changeFrame(comFrame);
      comForceSolution.setX(comWrenchVector.get(3, 0));
      comForceSolution.setY(comWrenchVector.get(4, 0));
      comForceSolution.setZ(comWrenchVector.get(5, 0));
   }

   public void getContactForceSolution(QuadrantDependentList<FrameVector> contactForce)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactForce.get(robotQuadrant).setIncludingFrame(contactForceSolution.get(robotQuadrant));
      }
   }

   public void getComTorqueSolution(FrameVector comTorque)
   {
      comTorque.setIncludingFrame(comTorqueSolution);
   }

   public void getComForceSolution(FrameVector comForce)
   {
      comForce.setIncludingFrame(comForceSolution);
   }

   private int getNumberOfContacts(QuadrantDependentList<boolean[]> contactState)
   {
      int numberOfContacts = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant)[0])
         {
            numberOfContacts++;
         }
      }
      return numberOfContacts;
   }

   private void initializeComWrenchMapMatrix(QuadrantDependentList<FramePoint> solePosition, QuadrantDependentList<boolean[]> contactState)
   {
      // compute map from contact forces to centroidal forces and torques
      int numberOfContacts = getNumberOfContacts(contactState);
      comWrenchMapMatrix.zero();
      comWrenchMapMatrix.reshape(6, 3 * numberOfContacts);
      int columnOffset = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactPosition.get(robotQuadrant).setIncludingFrame(solePosition.get(robotQuadrant));
         contactPosition.get(robotQuadrant).changeFrame(comFrame);
         if (contactState.get(robotQuadrant)[0])
         {
            comWrenchMapMatrix.set(0, 1 + columnOffset, -contactPosition.get(robotQuadrant).getZ()); // mX row
            comWrenchMapMatrix.set(0, 2 + columnOffset, contactPosition.get(robotQuadrant).getY());
            comWrenchMapMatrix.set(1, 0 + columnOffset, contactPosition.get(robotQuadrant).getZ()); // mY row
            comWrenchMapMatrix.set(1, 2 + columnOffset, -contactPosition.get(robotQuadrant).getX());
            comWrenchMapMatrix.set(2, 0 + columnOffset, -contactPosition.get(robotQuadrant).getY()); // mZ row
            comWrenchMapMatrix.set(2, 1 + columnOffset, contactPosition.get(robotQuadrant).getX());
            comWrenchMapMatrix.set(3, 0 + columnOffset, 1.0); // fX row
            comWrenchMapMatrix.set(4, 1 + columnOffset, 1.0); // fY row
            comWrenchMapMatrix.set(5, 2 + columnOffset, 1.0); // fZ row
            columnOffset += 3;
         }
      }
   }

   private void initializeComWrenchVector(FrameVector comTorqueCommand, FrameVector comForceCommand)
   {
      comTorqueCommand.changeFrame(comFrame);
      comWrenchVector.set(0, 0, comTorqueCommand.getX());
      comWrenchVector.set(1, 0, comTorqueCommand.getY());
      comWrenchVector.set(2, 0, comTorqueCommand.getZ());
      comForceCommand.changeFrame(comFrame);
      comWrenchVector.set(3, 0, comForceCommand.getX());
      comWrenchVector.set(4, 0, comForceCommand.getY());
      comWrenchVector.set(5, 0, comForceCommand.getZ());
   }

   private void initializeContactForceOptimizationWeights(QuadrupedContactForceOptimizationSettings optimizationSettings, int numberOfContacts)
   {
      double[] comTorqueCommandWeights = optimizationSettings.getComTorqueCommandWeights();
      double[] comForceCommandWeights = optimizationSettings.getComForceCommandWeights();
      double contactForceRegularizationWeights = optimizationSettings.getContactForceRegularizationWeights();

      comWrenchWeightMatrix.reshape(6, 6);
      for (int i = 0; i < 3; i++)
      {
         comWrenchWeightMatrix.set(i, i, comTorqueCommandWeights[i]);
      }
      for (int i = 0; i < 3; i++)
      {
         comWrenchWeightMatrix.set(i + 3, i + 3, comForceCommandWeights[i]);
      }

      contactForceRegularizationWeightMatrix.reshape(3 * numberOfContacts, 3 * numberOfContacts);
      for (int i = 0; i < 3 * numberOfContacts; i++)
      {
         contactForceRegularizationWeightMatrix.set(i, i, contactForceRegularizationWeights);
      }
   }

   private void initializeContactForceVector(QuadrantDependentList<boolean[]> contactState)
   {
      int numberOfContacts = getNumberOfContacts(contactState);
      contactForceVector.reshape(3 * numberOfContacts, 1);
   }

   private void computeUnconstrainedContactForceSolution()
   {
      // min_u 1/2 * (Mu - w)'Q(Mu - w) + 1/2 * u'Ru

      // output
      DenseMatrix64F u = contactForceVector;

      // inputs
      DenseMatrix64F w = comWrenchVector;
      DenseMatrix64F M = comWrenchMapMatrix;
      DenseMatrix64F Q = comWrenchWeightMatrix;
      DenseMatrix64F R = contactForceRegularizationWeightMatrix;

      // temporary
      DenseMatrix64F b = constraintVector;
      DenseMatrix64F A = constraintMatrix;
      DenseMatrix64F MtQM = temporaryMatrixA;
      DenseMatrix64F MtQ = temporaryMatrixB;

      int n = w.getNumRows();
      int m = u.getNumRows();

      // b = M'Qw
      MtQ.reshape(m, n);
      CommonOps.multTransA(M, Q, MtQ);
      b.reshape(m, 1);
      CommonOps.mult(MtQ, w, b);

      // A = M'QM + R
      MtQM.reshape(m, m);
      CommonOps.mult(MtQ, M, MtQM);
      A.reshape(m, m);
      CommonOps.add(MtQM, R, A);

      // u = inverse(A)*b
      CommonOps.invert(A);
      CommonOps.mult(A, b, u);
   }

   private void computeConstrainedContactForceSolution()
   {
      // TODO
      // compute joint torque inequality constraints
      // compute friction pyramid inequality constraints
      // compute min / max contact pressure constraints
      // compute contact forces using linear constrained quadratic program

   }

   private void computeComWrenchSolution()
   {
      CommonOps.mult(comWrenchMapMatrix, contactForceVector, comWrenchVector);
   }

   private void limitContactForceSolution(QuadrupedContactForceLimits contactForceLimits, QuadrantDependentList<boolean[]> contactState)
   {
      int rowOffset = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant)[0])
         {
            double mu = contactForceLimits.getCoefficientOfFriction(robotQuadrant);
            double fx = contactForceVector.get(0 + rowOffset, 0);
            double fy = contactForceVector.get(1 + rowOffset, 0);
            double fz = contactForceVector.get(2 + rowOffset, 0);

            // apply contact force limits
            fz = Math.max(fz, contactForceLimits.getPressureLowerLimit(robotQuadrant));
            fz = Math.min(fz, contactForceLimits.getPressureUpperLimit(robotQuadrant));
            fx = Math.max(fx, -mu * fz / Math.sqrt(2));
            fx = Math.min(fx, mu * fz / Math.sqrt(2));
            fy = Math.max(fy, -mu * fz / Math.sqrt(2));
            fy = Math.min(fy, mu * fz / Math.sqrt(2));

            contactForceVector.set(0 + rowOffset, 0, fx);
            contactForceVector.set(1 + rowOffset, 0, fy);
            contactForceVector.set(2 + rowOffset, 0, fz);

            rowOffset += 3;
         }
      }
   }
}
