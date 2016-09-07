package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.SdfLoader.models.FullQuadrupedRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
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

   private final QuadrantDependentList<FrameVector> soleForce;
   private final QuadrantDependentList<FrameVector> soleContactForce;
   private final QuadrantDependentList<FramePoint> solePosition;
   private final QuadrantDependentList<FrameVector[]> jointTorques;
   private final QuadrantDependentList<YoFrameVector> yoSoleForce;
   private final QuadrantDependentList<YoFrameVector> yoSoleContactForce;
   private final QuadrantDependentList<YoFramePoint> yoSolePosition;
   private final QuadrantDependentList<YoFrameVector[]> yoJointTorques;

   private final QuadrantDependentList<OneDoFJoint[]> legJoints;
   private final LegJointName[] legJointNames;
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
      legJointNames = fullRobotModel.getRobotSpecificJointNames().getLegJointNames();
      registry = new YoVariableRegistry(getClass().getSimpleName());

      // initialize reference frames
      this.referenceFrames = referenceFrames;
      worldFrame = ReferenceFrame.getWorldFrame();
      soleFrame = referenceFrames.getFootReferenceFrames();

      // initialize control variables
      solePosition = new QuadrantDependentList<>();
      soleForce = new QuadrantDependentList<>();
      soleContactForce = new QuadrantDependentList<>();
      jointTorques = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.set(robotQuadrant, new FramePoint(worldFrame));
         soleForce.set(robotQuadrant, new FrameVector(worldFrame));
         soleContactForce.set(robotQuadrant, new FrameVector(worldFrame));
         jointTorques.set(robotQuadrant, new FrameVector[legJointNames.length]);
         for (int i = 0; i < legJointNames.length; i++)
         {
            jointTorques.get(robotQuadrant)[i] = new FrameVector(worldFrame);
         }
      }

      // initialize yo variables
      yoSolePosition = new QuadrantDependentList<>();
      yoSoleForce = new QuadrantDependentList<>();
      yoSoleContactForce = new QuadrantDependentList<>();
      yoJointTorques = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoSolePosition.set(robotQuadrant, new YoFramePoint(robotQuadrant.getCamelCaseName() + "SolePosition", worldFrame, registry));
         yoSoleForce.set(robotQuadrant, new YoFrameVector(robotQuadrant.getCamelCaseName() + "SoleForceEstimate", worldFrame, registry));
         yoSoleForce.get(robotQuadrant).set(soleForce.get(robotQuadrant));
         yoSoleContactForce.set(robotQuadrant, new YoFrameVector(robotQuadrant.getCamelCaseName() + "SoleContactForceEstimate", worldFrame, registry));
         yoJointTorques.set(robotQuadrant, new YoFrameVector[legJointNames.length]);
         for (int i = 0; i < legJointNames.length; i++)
         {
            yoJointTorques.get(robotQuadrant)[i] = new YoFrameVector(robotQuadrant.getCamelCaseName() + legJointNames[i].getPascalCaseName() + "JointTorques",
                  worldFrame, registry);
         }
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
         int jacobianCols = fullRobotModel.getLegOneDoFJoints(robotQuadrant).size();
         int jacobianRows = 3;
         RigidBody body = fullRobotModel.getRootJoint().getSuccessor();
         RigidBody foot = jointBeforeFoot.getSuccessor();
         legJoints.set(robotQuadrant, ScrewTools.filterJoints(ScrewTools.createJointPath(body, foot), OneDoFJoint.class));
         footJacobian.set(robotQuadrant, new GeometricJacobian(legJoints.get(robotQuadrant), body.getBodyFixedFrame()));
         soleJacobian.set(robotQuadrant, new PointJacobian());
         jacobianMatrix.set(robotQuadrant, new DenseMatrix64F(jacobianRows, jacobianCols));
         jacobianMatrixTranspose.set(robotQuadrant, new DenseMatrix64F(jacobianMatrix.get(robotQuadrant).getNumCols(), jacobianMatrix.get(robotQuadrant).getNumRows()));
         jacobianMatrixTransposePseudoInverse.set(robotQuadrant, new DenseMatrix64F(jacobianMatrix.get(robotQuadrant).getNumRows(), jacobianMatrix.get(robotQuadrant).getNumCols()));
         soleForceMatrix.set(robotQuadrant, new DenseMatrix64F(jacobianMatrix.get(robotQuadrant).getNumRows(), 1));
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
         soleForce.get(robotQuadrant).setToZero();
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
            jointTorqueVector.get(robotQuadrant).set(i, 0, fullRobotModel.getLegOneDoFJoints(robotQuadrant).get(i).getTau());
         }
      }

      // Compute sole forces using jacobian transpose by
      // (J^T)^+ , where + is the pseudo-inverse operator
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         jacobianMatrix.set(robotQuadrant, soleJacobian.get(robotQuadrant).getJacobianMatrix());
         CommonOps.transpose(jacobianMatrix.get(robotQuadrant), jacobianMatrixTranspose.get(robotQuadrant));
         CommonOps.pinv(jacobianMatrixTranspose.get(robotQuadrant), jacobianMatrixTransposePseudoInverse.get(robotQuadrant));
         CommonOps.mult(jacobianMatrixTransposePseudoInverse.get(robotQuadrant), jointTorqueVector.get(robotQuadrant), soleForceMatrix.get(robotQuadrant));

         ReferenceFrame jacobianFrame = soleJacobian.get(robotQuadrant).getFrame();
         soleForce.get(robotQuadrant).changeFrame(jacobianFrame);
         soleContactForce.get(robotQuadrant).changeFrame(jacobianFrame);
         soleForce.get(robotQuadrant).set(soleForceMatrix.get(robotQuadrant).get(0), soleForceMatrix.get(robotQuadrant).get(1), soleForceMatrix.get(robotQuadrant).get(2));
         soleContactForce.get(robotQuadrant).set(-soleForceMatrix.get(robotQuadrant).get(0), -soleForceMatrix.get(robotQuadrant).get(1), -soleForceMatrix.get(robotQuadrant).get(2));
         yoSoleForce.get(robotQuadrant).setAndMatchFrame(soleForce.get(robotQuadrant));
         yoSoleContactForce.get(robotQuadrant).setAndMatchFrame(soleContactForce.get(robotQuadrant));
      }
   }

   public QuadrantDependentList<FrameVector> getSoleForce()
   {
      return soleForce;
   }

   public FrameVector getSoleForce(RobotQuadrant key)
   {
      return soleForce.get(key);
   }

   public QuadrantDependentList<FrameVector> getSoleContactForce()
   {
      return soleContactForce;
   }

   public FrameVector getSoleContactForce(RobotQuadrant key)
   {
      return soleContactForce.get(key);
   }
}
