package us.ihmc.commonWalkingControlModules.forcePolytope;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.convexOptimization.linearProgram.LinearProgramSolver;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
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
   private static final boolean debug = false;
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
   private final DMatrixRMaj inequalityA = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj inequalityB = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj cost = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj forceSolution = new DMatrixRMaj(3, 1);

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
      inequalityA.reshape(2 * joints.length, 6);
      inequalityB.reshape(2 * joints.length, 1);
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

      int rows = jacobianTranspose.getNumRows();
      int cols = jacobianTranspose.getNumCols();

      MatrixTools.setMatrixBlock(inequalityA, 0, 0, jacobianTranspose, 0, 0, rows, cols, 1.0);
      MatrixTools.setMatrixBlock(inequalityA, 0, cols, jacobianTranspose, 0, 0, rows, cols, -1.0);
      MatrixTools.setMatrixBlock(inequalityA, rows, 0, jacobianTranspose, 0, 0, rows, cols, -1.0);
      MatrixTools.setMatrixBlock(inequalityA, rows, cols, jacobianTranspose, 0, 0, rows, cols, 1.0);

      MatrixTools.setMatrixBlock(inequalityB, 0, 0, tauUpperLimit, 0, 0, joints.length, 1, 1.0);
      MatrixTools.setMatrixBlock(inequalityB, rows, 0, tauLowerLimit, 0, 0, joints.length, 1, -1.0);

      forcePolytope.clear();
      for (int i = 0; i < directionsToOptimize.length; i++)
      {
         Point3D direction = directionsToOptimize[i];
         for (int j = 0; j < 3; j++)
         {
            cost.set(j, direction.getElement(j));
            cost.set(3 + j, -direction.getElement(j));
         }

         if (lpSolver.solve(cost, inequalityA, inequalityB, forceSolution))
         {
            Point3D vertex = new Point3D();
            vertex.set(forceSolution.get(0), forceSolution.get(1), forceSolution.get(2));
            vertex.sub(forceSolution.get(3), forceSolution.get(4), forceSolution.get(5));
            forcePolytope.addVertex(vertex);
         }
      }

      if (debug)
      {
         LogTools.info("------- Force Polytope -------------");
         for (int i = 0; i < forcePolytope.getNumberOfVertices(); i++)
         {
            LogTools.info(forcePolytope.getVertex(i));
         }
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
}
