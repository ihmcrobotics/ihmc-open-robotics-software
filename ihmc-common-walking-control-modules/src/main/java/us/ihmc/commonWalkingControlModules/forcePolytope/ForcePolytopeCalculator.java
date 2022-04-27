package us.ihmc.commonWalkingControlModules.forcePolytope;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.convexOptimization.linearProgram.LinearProgramSolver;
import us.ihmc.convexOptimization.quadraticProgram.QuadProgSolver;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolverWithInactiveVariables;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;
import us.ihmc.robotics.screwTheory.PointJacobian;

public class ForcePolytopeCalculator
{
   private static final boolean solveWithLP = false;
   private static final boolean debug = true;
   private static final double gravity = -9.81;

   private final RigidBodyBasics rootBody;
   private final RigidBodyBasics base;
   private final RigidBodyBasics endEffector;
   private final OneDoFJointBasics[] joints;
   private final GravityCoriolisExternalWrenchMatrixCalculator calculator;

   private final ConvexPolytope3D forcePolytope = new ConvexPolytope3D();

   private final DMatrixRMaj tauLowerLimit = new DMatrixRMaj(0);
   private final DMatrixRMaj tauUpperLimit = new DMatrixRMaj(0);

   private final GeometricJacobian jacobian;
   private final PointJacobian pointJacobian = new PointJacobian();
   private final DMatrixRMaj jacobianTranspose = new DMatrixRMaj(0);
   private final DMatrixRMaj gravityCompensationTorques = new DMatrixRMaj(0);

   private final Point3D contactPointInParentFrameAfterJoint = new Point3D();
   private final FramePoint3D contactPoint = new FramePoint3D();
   private final FramePoint3D tempPoint = new FramePoint3D();
   private final ReferenceFrame contactPointFrame;
   private final Point3D[] directionsToOptimize;

   private final LinearProgramSolver lpSolver = new LinearProgramSolver();
   private final DMatrixRMaj lpInequalityA = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj lpInequalityB = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj lpCost = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj lpSolution = new DMatrixRMaj(3, 1);

//   private final SimpleEfficientActiveSetQPSolver qpSolver = new SimpleEfficientActiveSetQPSolver();
   private final DMatrixRMaj qpQuadraticCost = new DMatrixRMaj(0);
   private final DMatrixRMaj qpLinearCost = new DMatrixRMaj(0);
   private final DMatrixRMaj qpInequalityA = new DMatrixRMaj(0);
   private final DMatrixRMaj qpInequalityB = new DMatrixRMaj(0);
   private final DMatrixRMaj qpSolution = new DMatrixRMaj(0);

   private final DMatrixRMaj identity = new DMatrixRMaj(0);
   private final DMatrixRMaj jacobianOuterProduct = new DMatrixRMaj(0);

   public ForcePolytopeCalculator(RigidBodyBasics base, RigidBodyBasics endEffector)
   {
      this.rootBody = MultiBodySystemTools.getRootBody(base);
      this.base = base;
      this.endEffector = endEffector;
      this.joints = MultiBodySystemTools.createOneDoFJointPath(base, endEffector);
      this.calculator = new GravityCoriolisExternalWrenchMatrixCalculator(base, false);
      this.tauLowerLimit.reshape(joints.length, 1);
      this.tauUpperLimit.reshape(joints.length, 1);
      this.jacobian = new GeometricJacobian(base, endEffector, base.getBodyFixedFrame());
      this.contactPointFrame = new ReferenceFrame(endEffector.getName() + "ContactFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            tempPoint.setIncludingFrame(endEffector.getParentJoint().getFrameAfterJoint(), contactPointInParentFrameAfterJoint);
            tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
            transformToParent.setTranslationAndIdentityRotation(tempPoint);
         }
      };

      gravityCompensationTorques.reshape(joints.length, 1);
      jacobian.changeFrame(contactPointFrame);
      jacobianTranspose.reshape(joints.length, 3);
      calculator.setGravitionalAcceleration(gravity);
      directionsToOptimize = SpiralBasedAlgorithm.generatePointsOnSphere(1.0, 50);
      lpInequalityA.reshape(2 * joints.length, 6);
      lpInequalityB.reshape(2 * joints.length, 1);

      int qpSize = 3 + joints.length;
      qpQuadraticCost.reshape(qpSize, qpSize);
      qpLinearCost.reshape(qpSize, 1);
      qpInequalityA.reshape(2 * joints.length, qpSize);
      qpInequalityB.reshape(2 * joints.length, 1);
      qpSolution.reshape(qpSize, 1);
      identity.reshape(joints.length, joints.length);
      CommonOps_DDRM.setIdentity(identity);
      jacobianOuterProduct.reshape(3, 3);
   }

   public void update()
   {
      rootBody.updateFramesRecursively();
      contactPoint.setIncludingFrame(endEffector.getParentJoint().getFrameAfterJoint(), contactPointInParentFrameAfterJoint);

      /* Compute point jacobian */
      ReferenceFrame baseFrame = jacobian.getBaseFrame();
      jacobian.changeFrame(baseFrame);
      jacobian.compute();

      tempPoint.setIncludingFrame(endEffector.getParentJoint().getFrameAfterJoint(), contactPointInParentFrameAfterJoint);
      pointJacobian.set(jacobian, tempPoint);
      pointJacobian.compute();
      DMatrixRMaj jacobianMatrix = pointJacobian.getJacobianMatrix();
      CommonOps_DDRM.transpose(jacobianMatrix, jacobianTranspose);

      /* Compute point jacobian */
      calculator.compute();

      for (int i = 0; i < joints.length; i++)
      {
         gravityCompensationTorques.set(i, 0, calculator.getComputedJointTau(joints[i]).get(0));
      }

      for (int i = 0; i < joints.length; i++)
      {
         tauLowerLimit.set(i, -gravityCompensationTorques.get(i) + joints[i].getEffortLimitLower());
         tauUpperLimit.set(i, -gravityCompensationTorques.get(i) + joints[i].getEffortLimitUpper());
      }

      forcePolytope.clear();

      for (int i = 0; i < directionsToOptimize.length; i++)
      {
         Point3D direction = directionsToOptimize[i];

         if (solveWithLP)
         {
            // setup as linear program
            int rows = jacobianTranspose.getNumRows();
            int cols = jacobianTranspose.getNumCols();

            MatrixTools.setMatrixBlock(lpInequalityA, 0, 0, jacobianTranspose, 0, 0, rows, cols, 1.0);
            MatrixTools.setMatrixBlock(lpInequalityA, 0, cols, jacobianTranspose, 0, 0, rows, cols, -1.0);
            MatrixTools.setMatrixBlock(lpInequalityA, rows, 0, jacobianTranspose, 0, 0, rows, cols, -1.0);
            MatrixTools.setMatrixBlock(lpInequalityA, rows, cols, jacobianTranspose, 0, 0, rows, cols, 1.0);

            MatrixTools.setMatrixBlock(lpInequalityB, 0, 0, tauUpperLimit, 0, 0, joints.length, 1, 1.0);
            MatrixTools.setMatrixBlock(lpInequalityB, rows, 0, tauLowerLimit, 0, 0, joints.length, 1, -1.0);

            for (int j = 0; j < 3; j++)
            {
               lpCost.set(j, direction.getElement(j));
               lpCost.set(3 + j, -direction.getElement(j));
            }

            if (lpSolver.solve(lpCost, lpInequalityA, lpInequalityB, lpSolution))
            {
               Point3D vertex = new Point3D();
               vertex.set(lpSolution.get(0), lpSolution.get(1), lpSolution.get(2));
               vertex.sub(lpSolution.get(3), lpSolution.get(4), lpSolution.get(5));
               forcePolytope.addVertex(vertex);
            }
         }
         else
         {
            // setup as quadratic program
            CommonOps_DDRM.multOuter(jacobianMatrix, jacobianOuterProduct);
            MatrixTools.setMatrixBlock(qpQuadraticCost, 0, 0, identity, 0, 0, joints.length, joints.length, 1.0);
            MatrixTools.setMatrixBlock(qpQuadraticCost, 0, joints.length, jacobianTranspose, 0, 0, jacobianTranspose.getNumRows(), jacobianTranspose.getNumCols(), -1.0);
            MatrixTools.setMatrixBlock(qpQuadraticCost, joints.length, 0, jacobianMatrix, 0, 0, jacobianMatrix.getNumRows(), jacobianMatrix.getNumCols(), -1.0);
            MatrixTools.setMatrixBlock(qpQuadraticCost, joints.length, joints.length, jacobianOuterProduct, 0, 0, jacobianOuterProduct.getNumRows(), jacobianOuterProduct.getNumCols(), 1.0);

            double quadCostScale = 1.0;
            CommonOps_DDRM.scale(quadCostScale, qpQuadraticCost);

            direction.get(qpLinearCost);
            CommonOps_DDRM.scale(-1.0, qpLinearCost);

            MatrixTools.setMatrixBlock(qpInequalityA, 0, 0, identity, 0, 0, identity.getNumRows(), identity.getNumCols(), -1.0);
            MatrixTools.setMatrixBlock(qpInequalityA, identity.getNumRows(), 0, identity, 0, 0, identity.getNumRows(), identity.getNumCols(), 1.0);
            MatrixTools.setMatrixBlock(qpInequalityB, 0, 0, tauLowerLimit, 0, 0, tauLowerLimit.getNumRows(), tauLowerLimit.getNumCols(), -1.0);
            MatrixTools.setMatrixBlock(qpInequalityB, identity.getNumRows(), 0, tauUpperLimit, 0, 0, tauUpperLimit.getNumRows(), tauUpperLimit.getNumCols(), 1.0);

            for (int j = 0; j < 3; j++)
            {
               double forceDampingTerm = 1e-6;
               qpQuadraticCost.add(joints.length + j, joints.length + j, forceDampingTerm);
            }

//            SimpleEfficientActiveSetQPSolver qpSolver = new SimpleEfficientActiveSetQPSolver();
            SimpleEfficientActiveSetQPSolverWithInactiveVariables qpSolver = new SimpleEfficientActiveSetQPSolverWithInactiveVariables();

            qpSolver.setUseWarmStart(false);
            qpSolver.setMaxNumberOfIterations(1000);
            qpSolver.setConvergenceThreshold(1e-5);
            qpSolver.clear();
            qpSolver.resetActiveSet();
            qpSolver.setQuadraticCostFunction(qpQuadraticCost, qpLinearCost);
            qpSolver.setLinearInequalityConstraints(qpInequalityA, qpInequalityB);
            int iterations = qpSolver.solve(qpSolution);

            System.out.println(qpSolution);
            System.out.println("iterations: " + iterations);

            Point3D vertex = new Point3D();
            vertex.set(joints.length, qpSolution);

            if (!vertex.containsNaN())
               forcePolytope.addVertex(vertex);
         }
      }

      if (debug)
      {
         LogTools.info("------- Force Polytope -------------");
//         for (int i = 0; i < forcePolytope.getNumberOfVertices(); i++)
//         {
//            LogTools.info(forcePolytope.getVertex(i));
//         }
      }
   }

   public ConvexPolytope3D getForcePolytope()
   {
      return forcePolytope;
   }

   /**
    * Contact point, expressed in the parent joint's "after joint" frame
    */
   public void setContactPoint(Tuple3DReadOnly contactPoint)
   {
      this.contactPointInParentFrameAfterJoint.set(contactPoint);
   }

   public RigidBodyBasics getBase()
   {
      return base;
   }

   public RigidBodyBasics getEndEffector()
   {
      return endEffector;
   }

   public Point3D getContactPoint()
   {
      return contactPointInParentFrameAfterJoint;
   }
}
