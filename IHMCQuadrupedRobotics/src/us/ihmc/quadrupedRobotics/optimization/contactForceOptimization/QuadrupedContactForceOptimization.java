package us.ihmc.quadrupedRobotics.optimization.contactForceOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.convexOptimization.quadraticProgram.ConstrainedQPSolver;
import us.ihmc.convexOptimization.quadraticProgram.QuadProgSolver;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class QuadrupedContactForceOptimization
{
   private final ReferenceFrame comFrame;
   private final QuadrantDependentList<ReferenceFrame> soleFrame;
   private final ReferenceFrame worldFrame;
   private static final ConstrainedQPSolver qpSolver = new QuadProgSolver();

   private final FrameVector3D comTorqueCommand;
   private final FrameVector3D comTorqueSolution;
   private final FrameVector3D comForceCommand;
   private final FrameVector3D comForceSolution;
   private final QuadrantDependentList<FrameVector3D> contactForceCommand = new QuadrantDependentList<>();
   private final QuadrantDependentList<FrameVector3D> contactForceSolution = new QuadrantDependentList<>();
   private final QuadrantDependentList<FramePoint3D> contactPosition = new QuadrantDependentList<>();
   private final QuadrantDependentList<ContactState> contactState = new QuadrantDependentList<>();

   private final YoVariableRegistry registry;
   private final YoFrameVector yoComTorqueCommand;
   private final YoFrameVector yoComTorqueSolution;
   private final YoFrameVector yoComForceCommand;
   private final YoFrameVector yoComForceSolution;
   private final QuadrantDependentList<YoFrameVector> yoContactForceCommand = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFrameVector> yoContactForceSolution = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFramePoint> yoContactPosition = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoEnum<ContactState>> yoContactState = new QuadrantDependentList<>();

   private final DenseMatrix64F comWrenchCommandVector;
   private final DenseMatrix64F comWrenchSolutionVector;
   private final DenseMatrix64F comWrenchMapMatrix;
   private final DenseMatrix64F comWrenchWeightMatrix;
   private final DenseMatrix64F contactForceCommandVector;
   private final DenseMatrix64F contactForceSolutionVector;
   private final DenseMatrix64F contactForceWeightMatrix;
   private final DenseMatrix64F qpCostVector;
   private final DenseMatrix64F qpCostMatrix;
   private final DenseMatrix64F qpEqualityVector;
   private final DenseMatrix64F qpEqualityMatrix;
   private final DenseMatrix64F qpInequalityVector;
   private final DenseMatrix64F qpInequalityMatrix;
   private final DenseMatrix64F temporaryMatrixA;
   private final DenseMatrix64F temporaryMatrixB;

   public QuadrupedContactForceOptimization(QuadrupedReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(getClass().getSimpleName());
      comFrame = referenceFrames.getCenterOfMassZUpFrame();
      soleFrame = referenceFrames.getFootReferenceFrames();
      worldFrame = ReferenceFrame.getWorldFrame();

      comTorqueCommand = new FrameVector3D(comFrame);
      comTorqueSolution = new FrameVector3D(comFrame);
      comForceCommand = new FrameVector3D(comFrame);
      comForceSolution = new FrameVector3D(comFrame);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactForceCommand.set(robotQuadrant, new FrameVector3D(comFrame));
         contactForceSolution.set(robotQuadrant, new FrameVector3D(comFrame));
         contactPosition.set(robotQuadrant, new FramePoint3D(comFrame));
         contactState.set(robotQuadrant, ContactState.NO_CONTACT);
      }

      yoComTorqueCommand = new YoFrameVector("ComTorqueCommand", worldFrame, registry);
      yoComTorqueSolution = new YoFrameVector("ComTorqueSolution", worldFrame, registry);
      yoComForceCommand = new YoFrameVector("ComForceCommand", worldFrame, registry);
      yoComForceSolution = new YoFrameVector("ComForceSolution", worldFrame, registry);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseName();
         yoContactForceCommand.set(robotQuadrant, new YoFrameVector(prefix + "ContactForceCommand", worldFrame, registry));
         yoContactForceSolution.set(robotQuadrant, new YoFrameVector(prefix + "ContactForceSolution", worldFrame, registry));
         yoContactPosition.set(robotQuadrant, new YoFramePoint(prefix + "ContactPosition", worldFrame, registry));
         yoContactState.set(robotQuadrant, new YoEnum<>(prefix + "ContactState", registry, ContactState.class));
      }

      comWrenchCommandVector = new DenseMatrix64F(6, 1);
      comWrenchSolutionVector = new DenseMatrix64F(6, 1);
      comWrenchMapMatrix = new DenseMatrix64F(6, 12);
      comWrenchWeightMatrix = new DenseMatrix64F(6, 6);
      contactForceCommandVector = new DenseMatrix64F(12, 1);
      contactForceSolutionVector = new DenseMatrix64F(12, 1);
      contactForceWeightMatrix = new DenseMatrix64F(12, 12);
      qpCostVector = new DenseMatrix64F(12, 1);
      qpCostMatrix = new DenseMatrix64F(12, 12);
      qpEqualityVector = new DenseMatrix64F(0, 1);
      qpEqualityMatrix = new DenseMatrix64F(0, 12);
      qpInequalityVector = new DenseMatrix64F(24, 1);
      qpInequalityMatrix = new DenseMatrix64F(24, 12);
      temporaryMatrixA = new DenseMatrix64F(12, 12);
      temporaryMatrixB = new DenseMatrix64F(12, 12);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      // initialize commands
      comForceCommand.setToZero();
      comTorqueCommand.setToZero();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactForceCommand.get(robotQuadrant).setToZero();
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
      }
   }

   public void setComTorqueCommand(FrameVector3D comTorque)
   {
      comTorqueCommand.setIncludingFrame(comTorque);
   }

   public void setComForceCommand(FrameVector3D comForce)
   {
      comForceCommand.setIncludingFrame(comForce);
   }

   public void setContactForceCommand(RobotQuadrant robotQuadrant, FrameVector3D contactForce)
   {
      contactForceCommand.get(robotQuadrant).setIncludingFrame(contactForce);
   }

   public void setContactState(RobotQuadrant robotQuadrant, ContactState state)
   {
      contactState.set(robotQuadrant, state);
   }

   public ContactState getContactState(RobotQuadrant robotQuadrant)
   {
      return contactState.get(robotQuadrant);
   }

   public void solve(QuadrupedContactForceLimits contactForceLimits, QuadrupedContactForceOptimizationSettings optimizationSettings)
   {
      // initialize optimization variables
      initializeComWrenchMapMatrix();
      initializeComWrenchCommandVector();
      initializeContactForceCommandVector();
      initializeOptimizationWeights(optimizationSettings);
      initializeOptimizationCostTerms();
      initializeOptimizationConstraints(contactForceLimits);

      // compute contact force solution
      if (optimizationSettings.getSolver() == QuadrupedContactForceOptimizationSettings.Solver.CONSTRAINED_QP)
      {
         computeConstrainedContactForceSolution();
      }
      else
      {
         computeUnconstrainedContactForceSolution();
         constrainContactForceSolution(contactForceLimits);
      }
      int rowOffset = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            contactForceSolution.get(robotQuadrant).changeFrame(comFrame);
            contactForceSolution.get(robotQuadrant).setX(contactForceSolutionVector.get(0 + rowOffset, 0));
            contactForceSolution.get(robotQuadrant).setY(contactForceSolutionVector.get(1 + rowOffset, 0));
            contactForceSolution.get(robotQuadrant).setZ(contactForceSolutionVector.get(2 + rowOffset, 0));

            rowOffset += 3;
         }
         else
         {
            contactForceSolution.get(robotQuadrant).changeFrame(comFrame);
            contactForceSolution.get(robotQuadrant).setToZero();
         }
      }

      // compute com wrench solution
      computeComWrenchSolution();
      comTorqueSolution.changeFrame(comFrame);
      comTorqueSolution.setX(comWrenchSolutionVector.get(0, 0));
      comTorqueSolution.setY(comWrenchSolutionVector.get(1, 0));
      comTorqueSolution.setZ(comWrenchSolutionVector.get(2, 0));
      comForceSolution.changeFrame(comFrame);
      comForceSolution.setX(comWrenchSolutionVector.get(3, 0));
      comForceSolution.setY(comWrenchSolutionVector.get(4, 0));
      comForceSolution.setZ(comWrenchSolutionVector.get(5, 0));

      // update yo variables
      yoComTorqueCommand.setAndMatchFrame(comTorqueCommand);
      yoComTorqueSolution.setAndMatchFrame(comTorqueSolution);
      yoComForceCommand.setAndMatchFrame(comForceCommand);
      yoComForceSolution.setAndMatchFrame(comForceSolution);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoContactForceCommand.get(robotQuadrant).setAndMatchFrame(contactForceCommand.get(robotQuadrant));
         yoContactForceSolution.get(robotQuadrant).setAndMatchFrame(contactForceSolution.get(robotQuadrant));
         yoContactPosition.get(robotQuadrant).setAndMatchFrame(contactPosition.get(robotQuadrant));
         yoContactState.get(robotQuadrant).set(contactState.get(robotQuadrant));
      }
   }

   public void getContactForceSolution(RobotQuadrant robotQuadrant, FrameVector3D contactForce)
   {
      contactForce.setIncludingFrame(contactForceSolution.get(robotQuadrant));
   }

   public void getComTorqueSolution(FrameVector3D comTorque)
   {
      comTorque.setIncludingFrame(comTorqueSolution);
   }

   public void getComForceSolution(FrameVector3D comForce)
   {
      comForce.setIncludingFrame(comForceSolution);
   }

   private int getNumberOfContacts()
   {
      int numberOfContacts = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            numberOfContacts++;
         }
      }
      return numberOfContacts;
   }

   private void initializeComWrenchMapMatrix()
   {
      // compute map from contact forces to centroidal forces and torques
      int numberOfContacts = getNumberOfContacts();
      comWrenchMapMatrix.reshape(6, 3 * numberOfContacts);
      comWrenchMapMatrix.zero();
      int columnOffset = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactPosition.get(robotQuadrant).setToZero(soleFrame.get(robotQuadrant));
         contactPosition.get(robotQuadrant).changeFrame(comFrame);
         if (contactState.get(robotQuadrant) == ContactState.IN_CONTACT)
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

   private void initializeComWrenchCommandVector()
   {
      comTorqueCommand.changeFrame(comFrame);
      comWrenchCommandVector.set(0, 0, comTorqueCommand.getX());
      comWrenchCommandVector.set(1, 0, comTorqueCommand.getY());
      comWrenchCommandVector.set(2, 0, comTorqueCommand.getZ());
      comForceCommand.changeFrame(comFrame);
      comWrenchCommandVector.set(3, 0, comForceCommand.getX());
      comWrenchCommandVector.set(4, 0, comForceCommand.getY());
      comWrenchCommandVector.set(5, 0, comForceCommand.getZ());
   }

   private void initializeContactForceCommandVector()
   {
      int numberOfContacts = getNumberOfContacts();

      contactForceCommandVector.reshape(3 * numberOfContacts, 1);
      contactForceCommandVector.zero();
      int rowOffset = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactForceCommand.get(robotQuadrant).changeFrame(comFrame);
         if (contactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            contactForceCommandVector.set(rowOffset++, 0, contactForceCommand.get(robotQuadrant).getX());
            contactForceCommandVector.set(rowOffset++, 0, contactForceCommand.get(robotQuadrant).getY());
            contactForceCommandVector.set(rowOffset++, 0, contactForceCommand.get(robotQuadrant).getZ());
         }
      }
      contactForceSolutionVector.reshape(3 * numberOfContacts, 1);
      contactForceSolutionVector.zero();
   }

   private void initializeOptimizationWeights(QuadrupedContactForceOptimizationSettings optimizationSettings)
   {
      int numberOfContacts = getNumberOfContacts();

      comWrenchWeightMatrix.reshape(6, 6);
      comWrenchWeightMatrix.zero();
      for (int i = 0; i < 3; i++)
      {
         double[] comTorqueCommandWeights = optimizationSettings.getComTorqueCommandWeights();
         comWrenchWeightMatrix.set(i, i, comTorqueCommandWeights[i]);
      }
      for (int i = 0; i < 3; i++)
      {
         double[] comForceCommandWeights = optimizationSettings.getComForceCommandWeights();
         comWrenchWeightMatrix.set(i + 3, i + 3, comForceCommandWeights[i]);
      }

      contactForceWeightMatrix.reshape(3 * numberOfContacts, 3 * numberOfContacts);
      contactForceWeightMatrix.zero();
      int rowOffset = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         double[] contactForceCommandWeights = optimizationSettings.getContactForceCommandWeights(robotQuadrant);
         if (contactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            for (int i = 0; i < 3; i++)
            {
               contactForceWeightMatrix.set(rowOffset + i, rowOffset + i, contactForceCommandWeights[i]);
            }
            rowOffset += 3;
         }
      }
   }

   private void initializeOptimizationCostTerms()
   {
      // min_u 1/2 * (Mu - w)'Q(Mu - w) + 1/2 * (u - v)'R(u - v), where Q >= 0, R > 0

      // output
      DenseMatrix64F u = contactForceSolutionVector;

      // inputs
      DenseMatrix64F v = contactForceCommandVector;
      DenseMatrix64F w = comWrenchCommandVector;
      DenseMatrix64F M = comWrenchMapMatrix;
      DenseMatrix64F Q = comWrenchWeightMatrix;
      DenseMatrix64F R = contactForceWeightMatrix;

      int n = w.getNumRows();
      int m = u.getNumRows();

      // min_u 1/2 * u'Au + b'u

      // temporary
      DenseMatrix64F MtQM = temporaryMatrixA;
      DenseMatrix64F MtQ = temporaryMatrixB;

      // cost terms
      DenseMatrix64F b = qpCostVector;
      DenseMatrix64F A = qpCostMatrix;

      // b = -M'Qw - Rv
      MtQ.reshape(m, n);
      CommonOps.multTransA(M, Q, MtQ);
      b.reshape(m, 1);
      CommonOps.mult(MtQ, w, b);
      CommonOps.multAdd(R, v, b);
      CommonOps.scale(-1, b);

      // A = M'QM + R
      MtQM.reshape(m, m);
      CommonOps.mult(MtQ, M, MtQM);
      A.reshape(m, m);
      CommonOps.add(MtQM, R, A);
   }

   private void initializeOptimizationConstraints(QuadrupedContactForceLimits contactForceLimits)
   {
      DenseMatrix64F u = contactForceSolutionVector;
      DenseMatrix64F beq = qpEqualityVector;
      DenseMatrix64F Aeq = qpEqualityMatrix;
      DenseMatrix64F bin = qpInequalityVector;
      DenseMatrix64F Ain = qpInequalityMatrix;

      int m = u.getNumRows();

      // equality constraints:
      // Aeq u = beq
      beq.reshape(0, 1);
      Aeq.reshape(0, m);

      // inequality constraints:
      // Ain u <= bin
      bin.reshape(2 * m, 1);
      Ain.reshape(2 * m, m);
      bin.zero();
      Ain.zero();

      int rowOffset = 0;
      int columnOffset = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            // compute inequality constraints to enforce pressure limits and friction pyramids
            double mu = contactForceLimits.getCoefficientOfFriction(robotQuadrant);

            // fx lower limit
            Ain.set(rowOffset, 0 + columnOffset, -1);
            Ain.set(rowOffset, 2 + columnOffset, -mu / Math.sqrt(2));
            bin.set(rowOffset, 0, 0);
            rowOffset++;

            // fx upper limit
            Ain.set(rowOffset, 0 + columnOffset, 1);
            Ain.set(rowOffset, 2 + columnOffset, -mu / Math.sqrt(2));
            bin.set(rowOffset, 0, 0);
            rowOffset++;

            // fy lower limit
            Ain.set(rowOffset, 1 + columnOffset, -1);
            Ain.set(rowOffset, 2 + columnOffset, -mu / Math.sqrt(2));
            bin.set(rowOffset, 0, 0);
            rowOffset++;

            // fy upper limit
            Ain.set(rowOffset, 1 + columnOffset, 1);
            Ain.set(rowOffset, 2 + columnOffset, -mu / Math.sqrt(2));
            bin.set(rowOffset, 0, 0);
            rowOffset++;

            // fz lower limit
            Ain.set(rowOffset, 2 + columnOffset, -1);
            bin.set(rowOffset, 0, -contactForceLimits.getPressureLowerLimit(robotQuadrant));
            rowOffset++;

            // fz upper limit
            Ain.set(rowOffset, 2 + columnOffset, 1);
            bin.set(rowOffset, 0, contactForceLimits.getPressureUpperLimit(robotQuadrant));
            rowOffset++;

            columnOffset += 3;
         }
      }
   }

   private void computeConstrainedContactForceSolution()
   {
      if (getNumberOfContacts() == 0)
      {
         return;
      }

      // min_u 1/2 * u'Au + b'u
      // s.t. Ain u + bin >= 0
      DenseMatrix64F u = contactForceSolutionVector;
      DenseMatrix64F b = qpCostVector;
      DenseMatrix64F A = qpCostMatrix;
      DenseMatrix64F beq = qpEqualityVector;
      DenseMatrix64F Aeq = qpEqualityMatrix;
      DenseMatrix64F bin = qpInequalityVector;
      DenseMatrix64F Ain = qpInequalityMatrix;

      // solve constrained quadratic program
      try
      {
         qpSolver.solve(A, b, Aeq, beq, Ain, bin, u, false);
      }
      catch (NoConvergenceException e)
      {
         System.err.println("NoConvergenceException: " + e.getMessage());
      }
   }

   private void computeUnconstrainedContactForceSolution()
   {
      if (getNumberOfContacts() == 0)
      {
         return;
      }

      // min_u 1/2 * u'Au + b'u
      DenseMatrix64F u = contactForceSolutionVector;
      DenseMatrix64F b = qpCostVector;
      DenseMatrix64F A = qpCostMatrix;

      // compute least squares solution:
      // u = inverse(A) * -b, where A > 0
      CommonOps.invert(A);
      CommonOps.scale(-1, b);
      CommonOps.mult(A, b, u);
      CommonOps.scale(-1, b);
   }

   private void constrainContactForceSolution(QuadrupedContactForceLimits contactForceLimits)
   {
      int rowOffset = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            double mu = contactForceLimits.getCoefficientOfFriction(robotQuadrant);
            double fx = contactForceSolutionVector.get(0 + rowOffset, 0);
            double fy = contactForceSolutionVector.get(1 + rowOffset, 0);
            double fz = contactForceSolutionVector.get(2 + rowOffset, 0);

            // apply contact force limits
            fz = Math.max(fz, contactForceLimits.getPressureLowerLimit(robotQuadrant));
            fz = Math.min(fz, contactForceLimits.getPressureUpperLimit(robotQuadrant));
            fx = Math.max(fx, -mu * fz / Math.sqrt(2));
            fx = Math.min(fx, mu * fz / Math.sqrt(2));
            fy = Math.max(fy, -mu * fz / Math.sqrt(2));
            fy = Math.min(fy, mu * fz / Math.sqrt(2));

            contactForceSolutionVector.set(0 + rowOffset, 0, fx);
            contactForceSolutionVector.set(1 + rowOffset, 0, fy);
            contactForceSolutionVector.set(2 + rowOffset, 0, fz);

            rowOffset += 3;
         }
      }
   }

   private void computeComWrenchSolution()
   {
      CommonOps.mult(comWrenchMapMatrix, contactForceSolutionVector, comWrenchSolutionVector);
   }
}
