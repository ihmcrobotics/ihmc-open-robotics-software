package us.ihmc.commonWalkingControlModules.forcePolytope;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.convexOptimization.exceptions.NoConvergenceException;
import us.ihmc.convexOptimization.linearProgram.LinearProgramSolver;
import us.ihmc.convexOptimization.quadraticProgram.QuadProgSolver;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
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

import java.util.ArrayList;
import java.util.List;

public class ForcePolytopeCalculator
{
   private static final SolveMethod solveMethod = SolveMethod.SVD;
   private static final boolean debug = true;
   private static final double gravity = -9.81;

   private enum SolveMethod
   {
      LP_INTERIOR_POINT,
      QP_INTERIOR_POINT,
      SVD
   }

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

   private final LPInteriorPointForcePolytopeSolver lpSolver;
   private final QPInteriorPointForcePolytopeSolver qpSolver;
   private final SVDForcePolytopeSolver svdSolver;

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

      lpSolver = new LPInteriorPointForcePolytopeSolver(joints.length);
      qpSolver = new QPInteriorPointForcePolytopeSolver(joints.length);
      svdSolver = new SVDForcePolytopeSolver(joints.length);
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

      if (solveMethod == SolveMethod.LP_INTERIOR_POINT)
      {
         lpSolver.solve(jacobianMatrix, tauLowerLimit, tauUpperLimit, forcePolytope);
      }
      else if (solveMethod == SolveMethod.QP_INTERIOR_POINT)
      {
         qpSolver.solve(jacobianMatrix, tauLowerLimit, tauUpperLimit, forcePolytope);
      }
      else if (solveMethod == SolveMethod.SVD)
      {
         svdSolver.solve(jacobianMatrix, tauLowerLimit, tauUpperLimit, forcePolytope);
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

   public LPInteriorPointForcePolytopeSolver getLpSolver()
   {
      return lpSolver;
   }

   public QPInteriorPointForcePolytopeSolver getQpSolver()
   {
      return qpSolver;
   }

   public SVDForcePolytopeSolver getSvdSolver()
   {
      return svdSolver;
   }
}
