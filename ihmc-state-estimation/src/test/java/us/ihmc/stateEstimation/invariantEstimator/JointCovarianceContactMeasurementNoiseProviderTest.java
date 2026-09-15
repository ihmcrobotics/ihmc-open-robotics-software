package us.ihmc.stateEstimation.invariantEstimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.robotModels.FullRobotModelTestTools.RandomFullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.stateEstimation.jointLevel.OneDoFJointStateSource;

/**
 * Pins the {@code contact_fk_r} deployment path: {@code R = scale * J Sigma_q J^T} routed from the joint-level
 * filter's own covariance, replacing the historical constant isotropic R.
 *
 * <p>The load-bearing test here is the finite-difference one: the provider's internal Jacobian is never exposed,
 * so correctness is established by differencing the actual forward kinematics the measurement is computed from.
 * A frame mistake (sole vs pelvis, or a missing rotation) survives every "the number changed" assertion but not
 * that one.</p>
 */
public class JointCovarianceContactMeasurementNoiseProviderTest
{
   private static final long SEED = 90210L;
   private static final double CONSTANT_VARIANCE = 1.0e-4;
   private static final double FALLBACK_VARIANCE = 2.5e-5;

   /** Joint source with a caller-supplied dense Sigma_q over whatever joints it is asked about. */
   private static class StubJointSource implements OneDoFJointStateSource
   {
      private final boolean hasCovariance;
      private final double variance;

      StubJointSource(boolean hasCovariance, double variance)
      {
         this.hasCovariance = hasCovariance;
         this.variance = variance;
      }

      @Override
      public boolean hasCovariance()
      {
         return hasCovariance;
      }

      @Override
      public void packPositionCovariance(OneDoFJointBasics[] joints, double fallbackVariance, DMatrixRMaj toPack)
      {
         toPack.reshape(joints.length, joints.length);
         toPack.zero();
         for (int i = 0; i < joints.length; i++)
            toPack.set(i, i, variance);
      }

      @Override
      public void packVelocityCovariance(OneDoFJointBasics[] joints, double fallbackVariance, DMatrixRMaj toPack)
      {
         packPositionCovariance(joints, fallbackVariance, toPack);
      }

      @Override
      public boolean containsJoint(OneDoFJointBasics joint)
      {
         return true;
      }

      @Override
      public double getEstimatedJointPosition(OneDoFJointBasics joint)
      {
         return joint.getQ();
      }

      @Override
      public double getEstimatedJointVelocity(OneDoFJointBasics joint)
      {
         return joint.getQd();
      }
   }

   private static RandomFullHumanoidRobotModel randomModel()
   {
      Random random = new Random(SEED);
      RandomFullHumanoidRobotModel model = new RandomFullHumanoidRobotModel(random);
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, model.getOneDoFJoints());
      model.getRootJoint().setJointConfigurationToZero();
      model.getElevator().updateFramesRecursively();
      return model;
   }

   private static SideDependentList<RigidBodyBasics> feetOf(RandomFullHumanoidRobotModel model)
   {
      SideDependentList<RigidBodyBasics> feet = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
         feet.put(side, model.getFoot(side));
      return feet;
   }

   private static JointCovarianceContactMeasurementNoiseProvider provider(RandomFullHumanoidRobotModel model,
                                                                          OneDoFJointStateSource jointSource,
                                                                          double scale,
                                                                          double conditioningFloor)
   {
      return new JointCovarianceContactMeasurementNoiseProvider(model.getPelvis(),
                                                                 feetOf(model),
                                                                 model.getSoleFrames(),
                                                                 jointSource,
                                                                 FALLBACK_VARIANCE,
                                                                 scale,
                                                                 conditioningFloor,
                                                                 new ConstantContactMeasurementNoiseProvider(CONSTANT_VARIANCE));
   }

   /**
    * The one test that can catch a frame error. With Sigma_q = sigma^2 I, R must equal sigma^2 J J^T, where J is
    * d(sole position expressed in PELVIS)/dq -- recomputed here by actually perturbing each joint and running the
    * model's own forward kinematics, never by reusing the provider's Jacobian code.
    */
   @Test
   public void testTheRoutedCovarianceMatchesAFiniteDifferencedForwardKinematicsJacobian()
   {
      RandomFullHumanoidRobotModel model = randomModel();
      double jointVariance = 1.0e-6;
      JointCovarianceContactMeasurementNoiseProvider provider = provider(model, new StubJointSource(true, jointVariance), 1.0, 0.0);

      for (RobotSide side : RobotSide.values)
      {
         Matrix3D actual = new Matrix3D();
         provider.packContactCovariance(side, actual);

         DMatrixRMaj J = finiteDifferenceJacobian(model, side);
         DMatrixRMaj expected = new DMatrixRMaj(3, 3);
         CommonOps_DDRM.multTransB(jointVariance, J, J, expected);

         // RELATIVE tolerance, scaled by the largest entry: this compares an analytic Jacobian against a
         // central-difference one, whose accuracy is roundoff-limited at ~eps/delta, not machine epsilon. A frame
         // error -- the thing this test exists to catch -- is an O(1) relative discrepancy, thousands of times
         // larger than the 1e-8 allowed here, so the loose absolute number costs the test nothing.
         double magnitude = CommonOps_DDRM.elementMaxAbs(expected);
         for (int row = 0; row < 3; row++)
            for (int col = 0; col < 3; col++)
               assertEquals(expected.get(row, col), actual.getElement(row, col), 1.0e-8 * magnitude, side + " R(" + row + "," + col + ")");

         // Not vacuous: a leg Jacobian at a random configuration must produce a genuinely nonzero, anisotropic R.
         assertTrue(expected.get(0, 0) > 0.0 && expected.get(1, 1) > 0.0 && expected.get(2, 2) > 0.0, "R must be nonzero");
         assertNotEquals(expected.get(0, 0), expected.get(2, 2), 1.0e-12, "a real leg Jacobian gives an anisotropic R");
      }
   }

   /** d(p_sole in pelvis frame)/dq_i by central differences on the live model's own FK. */
   private static DMatrixRMaj finiteDifferenceJacobian(RandomFullHumanoidRobotModel model, RobotSide side)
   {
      ReferenceFrame pelvisFrame = model.getPelvis().getBodyFixedFrame();
      ReferenceFrame soleFrame = model.getSoleFrames().get(side);
      OneDoFJointBasics[] joints = MultiBodySystemTools.createOneDoFJointPath(model.getPelvis(), model.getFoot(side));

      DMatrixRMaj J = new DMatrixRMaj(3, joints.length);
      // ~eps^(1/3): the central-difference optimum, balancing O(delta^2) truncation against O(eps/delta) roundoff.
      double delta = 1.0e-5;
      for (int i = 0; i < joints.length; i++)
      {
         double q0 = joints[i].getQ();

         joints[i].setQ(q0 + delta);
         model.getElevator().updateFramesRecursively();
         FramePoint3D plus = new FramePoint3D(soleFrame);
         plus.changeFrame(pelvisFrame);

         joints[i].setQ(q0 - delta);
         model.getElevator().updateFramesRecursively();
         FramePoint3D minus = new FramePoint3D(soleFrame);
         minus.changeFrame(pelvisFrame);

         joints[i].setQ(q0);
         model.getElevator().updateFramesRecursively();

         J.set(0, i, (plus.getX() - minus.getX()) / (2.0 * delta));
         J.set(1, i, (plus.getY() - minus.getY()) / (2.0 * delta));
         J.set(2, i, (plus.getZ() - minus.getZ()) / (2.0 * delta));
      }
      return J;
   }

   /** The learned {@code contact_fk_r} channel must multiply the whole routed covariance, and only that. */
   @Test
   public void testTheLearnedScaleMultipliesTheRoutedCovarianceExactly()
   {
      RandomFullHumanoidRobotModel model = randomModel();
      double scale = 6.0;
      Matrix3D unscaled = new Matrix3D();
      Matrix3D scaled = new Matrix3D();
      provider(model, new StubJointSource(true, 1.0e-6), 1.0, 0.0).packContactCovariance(RobotSide.LEFT, unscaled);
      provider(model, new StubJointSource(true, 1.0e-6), scale, 0.0).packContactCovariance(RobotSide.LEFT, scaled);

      for (int row = 0; row < 3; row++)
         for (int col = 0; col < 3; col++)
            assertEquals(unscaled.getElement(row, col) * scale, scaled.getElement(row, col), 1.0e-16, "R(" + row + "," + col + ")");
      assertTrue(Math.abs(unscaled.getElement(0, 0)) > 0.0, "baseline must be nonzero or the scaling check is vacuous");
   }

   /** Before the joint filter has a covariance, the previously-shipping constant model must still be used. */
   @Test
   public void testItFallsBackToTheConstantProviderWhileTheJointFilterHasNoCovariance()
   {
      RandomFullHumanoidRobotModel model = randomModel();
      Matrix3D packed = new Matrix3D();
      provider(model, new StubJointSource(false, 1.0e-6), 3.0, 0.0).packContactCovariance(RobotSide.LEFT, packed);

      assertEquals(CONSTANT_VARIANCE, packed.getElement(0, 0), 0.0);
      assertEquals(CONSTANT_VARIANCE, packed.getElement(1, 1), 0.0);
      assertEquals(CONSTANT_VARIANCE, packed.getElement(2, 2), 0.0);
      assertEquals(0.0, packed.getElement(0, 1), 0.0, "the constant fallback is isotropic; the scale must not leak in");
   }

   /** The conditioning floor is a Java-side addition on the diagonal only, applied after the learned scale. */
   @Test
   public void testTheConditioningFloorIsAddedOnTheDiagonalAfterScalingAndDefaultsAway()
   {
      RandomFullHumanoidRobotModel model = randomModel();
      double floor = 1.0e-8;
      Matrix3D withoutFloor = new Matrix3D();
      Matrix3D withFloor = new Matrix3D();
      provider(model, new StubJointSource(true, 1.0e-6), 2.0, 0.0).packContactCovariance(RobotSide.LEFT, withoutFloor);
      provider(model, new StubJointSource(true, 1.0e-6), 2.0, floor).packContactCovariance(RobotSide.LEFT, withFloor);

      for (int row = 0; row < 3; row++)
      {
         for (int col = 0; col < 3; col++)
         {
            double expected = withoutFloor.getElement(row, col) + (row == col ? floor : 0.0);
            assertEquals(expected, withFloor.getElement(row, col), 1.0e-16, "R(" + row + "," + col + ")");
         }
      }
   }

   @Test
   public void testItRejectsNonPositiveOrNonFiniteConstructorArguments()
   {
      RandomFullHumanoidRobotModel model = randomModel();
      OneDoFJointStateSource source = new StubJointSource(true, 1.0e-6);
      assertThrows(IllegalArgumentException.class, () -> provider(model, source, 0.0, 0.0), "zero scale");
      assertThrows(IllegalArgumentException.class, () -> provider(model, source, Double.NaN, 0.0), "NaN scale");
      assertThrows(IllegalArgumentException.class, () -> provider(model, source, 1.0, -1.0), "negative floor");
   }
}
