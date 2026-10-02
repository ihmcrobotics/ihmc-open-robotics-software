package us.ihmc.stateEstimation.invariantEstimator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotModels.FullRobotModelTestTools.RandomFullHumanoidRobotModel;

/**
 * The kinematic velocity noise N_v = sigma^2 I + J_c Sigma_qd J_c^T is only right if J_c maps the joint rates to the
 * same quantity the measurement is built from: the velocity of the stationary point c relative to the pelvis,
 * expressed in the pelvis frame. This checks J_c qd against that quantity computed the way the estimator computes
 * the measurement (the sole frame's twist relative to the pelvis, taken at c), at random configurations, rates and
 * points.
 */
public class KinematicVelocityPointJacobianTest
{
   @Test
   public void testPointJacobianTimesJointRatesIsThePointVelocityTheMeasurementUses()
   {
      Random random = new Random(4242L);
      RandomFullHumanoidRobotModel model = new RandomFullHumanoidRobotModel(random);
      model.getRootJoint().setJointConfigurationToZero();
      model.getRootJoint().setJointTwistToZero();
      ReferenceFrame pelvisFrame = model.getPelvis().getBodyFixedFrame();

      for (int trial = 0; trial < 50; trial++)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, model.getOneDoFJoints());
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, model.getOneDoFJoints());
         model.getElevator().updateFramesRecursively();

         for (RobotSide side : RobotSide.values)
         {
            ReferenceFrame soleFrame = model.getSoleFrame(side);
            OneDoFJointBasics[] joints = MultiBodySystemTools.createOneDoFJointPath(model.getPelvis(), model.getFoot(side));
            GeometricJacobianCalculator calculator = new GeometricJacobianCalculator();
            calculator.setKinematicChain(joints);
            calculator.setJacobianFrame(soleFrame);

            Point3D c = new Point3D(random.nextDouble() * 0.2 - 0.1, random.nextDouble() * 0.1 - 0.05, random.nextDouble() * 0.02);
            RotationMatrix soleToPelvis = new RotationMatrix(soleFrame.getTransformToDesiredFrame(pelvisFrame).getRotation());
            DMatrixRMaj pointJacobian = new DMatrixRMaj(3, joints.length);
            InvariantEKFStateEstimator.computePointJacobian(calculator.getJacobianMatrix(), c, soleToPelvis, pointJacobian);

            DMatrixRMaj qd = new DMatrixRMaj(joints.length, 1);
            for (int i = 0; i < joints.length; i++)
               qd.set(i, 0, joints[i].getQd());
            DMatrixRMaj predicted = new DMatrixRMaj(3, 1);
            CommonOps_DDRM.mult(pointJacobian, qd, predicted);

            // As in InvariantEKFStateEstimator.updateKinematicVelocity.
            Twist relativeTwist = new Twist();
            ((us.ihmc.mecano.frames.MovingReferenceFrame) soleFrame).getTwistRelativeToOther((us.ihmc.mecano.frames.MovingReferenceFrame) pelvisFrame,
                                                                                            relativeTwist);
            FrameVector3D expected = new FrameVector3D();
            relativeTwist.getLinearVelocityAt(new FramePoint3D(soleFrame, c), expected);
            expected.changeFrame(pelvisFrame);

            assertEquals(expected.getX(), predicted.get(0), 1.0e-10);
            assertEquals(expected.getY(), predicted.get(1), 1.0e-10);
            assertEquals(expected.getZ(), predicted.get(2), 1.0e-10);
         }
      }
   }
}
