package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import org.ejml.alg.dense.misc.UnrolledInverseFromMinor;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.*;

public class QuadrupedSoleForceEstimator
{
   private final YoVariableRegistry registry;

   private final FullQuadrupedRobotModel fullRobotModel;
   private final QuadrupedReferenceFrames referenceFrames;
   private final ReferenceFrame worldFrame;
   private final QuadrantDependentList<ReferenceFrame> soleFrame;

   private final QuadrantDependentList<FrameVector3D> soleVirtualForce;
   private final QuadrantDependentList<FrameVector3D> soleContactForce;
   private final QuadrantDependentList<FramePoint3D> solePosition;
   private final QuadrantDependentList<YoFrameVector> yoSoleVirtualForce;
   private final QuadrantDependentList<YoFrameVector> yoSoleContactForce;

   private final QuadrantDependentList<OneDoFJoint[]> legJoints;
   private final QuadrantDependentList<GeometricJacobian> footJacobian;
   private final QuadrantDependentList<PointJacobian> soleJacobian;

   private final QuadrantDependentList<DenseMatrix64F>  jacobianMatrix;
   private final QuadrantDependentList<DenseMatrix64F> jointTorqueVector;
   private final QuadrantDependentList<DenseMatrix64F> jacobianMatrixTranspose;
   private final QuadrantDependentList<DenseMatrix64F> jacobianMatrixTransposePseudoInverse;
   private final QuadrantDependentList<DenseMatrix64F> soleForceMatrix;


   public QuadrupedSoleForceEstimator(FullQuadrupedRobotModel fullRobotModel, QuadrupedReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      registry = new YoVariableRegistry(getClass().getSimpleName());

      // initialize reference frames
      this.referenceFrames = referenceFrames;
      worldFrame = ReferenceFrame.getWorldFrame();
      soleFrame = referenceFrames.getFootReferenceFrames();

      // initialize estimate variables
      solePosition = new QuadrantDependentList<>();
      soleVirtualForce = new QuadrantDependentList<>();
      soleContactForce = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.set(robotQuadrant, new FramePoint3D(worldFrame));
         soleVirtualForce.set(robotQuadrant, new FrameVector3D(worldFrame));
         soleContactForce.set(robotQuadrant, new FrameVector3D(worldFrame));
      }

      // initialize yo variables
      yoSoleVirtualForce = new QuadrantDependentList<>();
      yoSoleContactForce = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoSoleVirtualForce.set(robotQuadrant, new YoFrameVector(robotQuadrant.getCamelCaseName() + "SoleVirtualForceEstimate", worldFrame, registry));
         yoSoleContactForce.set(robotQuadrant, new YoFrameVector(robotQuadrant.getCamelCaseName() + "SoleContactForceEstimate", worldFrame, registry));
      }

      // initialize jacobian variables
      legJoints = new QuadrantDependentList<>();
      footJacobian = new QuadrantDependentList<>();
      soleJacobian = new QuadrantDependentList<>();
      jointTorqueVector = new QuadrantDependentList<>();
      jacobianMatrix = new QuadrantDependentList<>();
      jacobianMatrixTranspose = new QuadrantDependentList<>();
      jacobianMatrixTransposePseudoInverse = new QuadrantDependentList<>();
      soleForceMatrix = new QuadrantDependentList<>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         OneDoFJoint jointBeforeFoot = fullRobotModel.getOneDoFJointBeforeFoot(robotQuadrant);
         RigidBody body = fullRobotModel.getRootJoint().getSuccessor();
         RigidBody foot = jointBeforeFoot.getSuccessor();
         legJoints.set(robotQuadrant, ScrewTools.filterJoints(ScrewTools.createJointPath(body, foot), OneDoFJoint.class));
         footJacobian.set(robotQuadrant, new GeometricJacobian(legJoints.get(robotQuadrant), body.getBodyFixedFrame()));
         soleJacobian.set(robotQuadrant, new PointJacobian());
         int jacobianRows = 3;
         int jacobianCols = legJoints.get(robotQuadrant).length;
         jacobianMatrix.set(robotQuadrant, new DenseMatrix64F(jacobianRows, jacobianCols));
         jacobianMatrixTranspose.set(robotQuadrant, new DenseMatrix64F(jacobianCols, jacobianRows));
         jacobianMatrixTransposePseudoInverse.set(robotQuadrant, new DenseMatrix64F(jacobianRows, jacobianCols));
         soleForceMatrix.set(robotQuadrant, new DenseMatrix64F(3, 1));
         jointTorqueVector.set(robotQuadrant, new DenseMatrix64F(legJoints.get(robotQuadrant).length, 1));
         fullRobotModel.getLegOneDoFJoints(robotQuadrant).get(0).getTau();
      }
      parentRegistry.addChild(registry);
      this.reset();
   }

   public void reset()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         soleVirtualForce.get(robotQuadrant).setToZero();
         soleContactForce.get(robotQuadrant).setToZero();
      }
   }

   public void compute()
   {
      // compute sole positions and jacobians
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.get(robotQuadrant).setToZero(soleFrame.get(robotQuadrant));
         footJacobian.get(robotQuadrant).compute();
         soleJacobian.get(robotQuadrant).set(footJacobian.get(robotQuadrant), solePosition.get(robotQuadrant));
         soleJacobian.get(robotQuadrant).compute();

         for (int i = 0; i < legJoints.get(robotQuadrant).length; ++i)
         {
            jointTorqueVector.get(robotQuadrant).set(i, 0, legJoints.get(robotQuadrant)[i].getTauMeasured());
         }
      }

      // Compute sole forces using jacobian transpose by
      // (J^T)^+ , where + is the pseudo-inverse operator
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         jacobianMatrix.set(robotQuadrant, soleJacobian.get(robotQuadrant).getJacobianMatrix());
         CommonOps.transpose(jacobianMatrix.get(robotQuadrant), jacobianMatrixTranspose.get(robotQuadrant));
         UnrolledInverseFromMinor.inv3(jacobianMatrixTranspose.get(robotQuadrant), jacobianMatrixTransposePseudoInverse.get(robotQuadrant),1.0);
//         CommonOps.pinv(jacobianMatrixTranspose.get(robotQuadrant), jacobianMatrixTransposePseudoInverse.get(robotQuadrant));
         CommonOps.mult(jacobianMatrixTransposePseudoInverse.get(robotQuadrant), jointTorqueVector.get(robotQuadrant), soleForceMatrix.get(robotQuadrant));

         ReferenceFrame jacobianFrame = soleJacobian.get(robotQuadrant).getFrame();
         soleVirtualForce.get(robotQuadrant).changeFrame(jacobianFrame);
         soleContactForce.get(robotQuadrant).changeFrame(jacobianFrame);
         soleVirtualForce.get(robotQuadrant)
               .set(soleForceMatrix.get(robotQuadrant).get(0), soleForceMatrix.get(robotQuadrant).get(1), soleForceMatrix.get(robotQuadrant).get(2));
         soleContactForce.get(robotQuadrant)
               .set(-soleForceMatrix.get(robotQuadrant).get(0), -soleForceMatrix.get(robotQuadrant).get(1), -soleForceMatrix.get(robotQuadrant).get(2));
         yoSoleVirtualForce.get(robotQuadrant).setAndMatchFrame(soleVirtualForce.get(robotQuadrant));
         yoSoleContactForce.get(robotQuadrant).setAndMatchFrame(soleContactForce.get(robotQuadrant));
      }
   }

   public QuadrantDependentList<FrameVector3D> getSoleVirtualForce()
   {
      return soleVirtualForce;
   }

   public FrameVector3D getSoleVirtualForce(RobotQuadrant robotQuadrant)
   {
      return soleVirtualForce.get(robotQuadrant);
   }

   public QuadrantDependentList<FrameVector3D> getSoleContactForce()
   {
      return soleContactForce;
   }

   public FrameVector3D getSoleContactForce(RobotQuadrant robotQuadrant)
   {
      return soleContactForce.get(robotQuadrant);
   }
}
