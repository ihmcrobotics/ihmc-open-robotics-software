package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

/**
 * The direct-velocity channel's per-joint gate, and the stuck-sensor case it exists for.
 *
 * <h2>Why a frozen reading is worse than a missing one</h2>
 * <p>The channel's adaptive R inflates with the measurement's OWN slew
 * ({@code R = sigma^2 + (slew/omega)^2}). A frozen signal has zero slew, so R sits at its noise floor and the
 * filter trusts it completely — precisely when it is most wrong. A dropout is finite-checked and skipped, so
 * before this gate existed a sensor that reported nothing was handled correctly while a sensor that reported a
 * stale value silently collapsed that joint's velocity estimate. These tests pin that inversion closed.</p>
 *
 * <h2>Why staleness needs the encoder</h2>
 * <p>A joint truly at rest reports the same value every tick too, so a rule that drops any long identical run
 * would disable the channel throughout quiet stance — the regime it exists to sharpen. The gate therefore calls
 * a frozen q̇ stale only when the joint's own encoder says it travelled further than that q̇ can account for.
 * {@link #testAJointGenuinelyAtRestKeepsItsVelocityRow()} is the half of the contract that would fail under the
 * naive rule.</p>
 */
public class JointLevelKFStuckVelocityTest
{
   private static final int TICKS = 1500;

   /** The fixture variant with the direct-velocity channel ENABLED — the channel under test here. Nominal
    *  noise lookups; a NaN break frequency means no lag inflation, which keeps R at its floor and makes the
    *  stuck reading maximally dominant, i.e. the hardest case for the gate. */
   private static JointLevelKFTestFixture directVelocityFixture(long seed, int numJoints, int parent, int child)
   {
      return JointLevelKFTestFixture.singlePairWithDirectVelocity(seed, numJoints, parent, child,
                                                                   name -> Double.NaN,
                                                                   name -> Double.NaN,
                                                                   name -> Double.NaN);
   }
   private static final double TRUE_QD = 0.25; // rad/s, well above any tolerance

   /** Drives the fixture with a constant true velocity while the firmware velocity reports {@code reported}. */
   private static double estimatedVelocityAfterRun(JointLevelKFTestFixture fixture, double trueQd, double reported)
   {
      int n = fixture.filteredJoints.size();
      double[] qTrue = new double[n];
      double[] qdTrue = new double[n];
      for (int i = 0; i < n; i++)
         qdTrue[i] = trueQd;

      List<RigidBodyBasics> noTrustedFeet = new ArrayList<>();
      for (int tick = 0; tick < TICKS; tick++)
      {
         for (int i = 0; i < n; i++)
            qTrue[i] += qdTrue[i] * JointLevelKFTestFixture.DT;
         fixture.applyConsistentMotion(qTrue, qdTrue);
         for (OneDoFJointBasics joint : fixture.filteredJoints)
            fixture.setMeasuredVelocity(joint, reported);
         fixture.filter.computeJointState();
         fixture.filter.computeImuBiases(noTrustedFeet);
      }
      return fixture.filter.getEstimatedJointVelocity(fixture.filteredJoints.get(0));
   }

   /**
    * The headline case. A firmware velocity frozen at zero while the joint really moves must not drag the
    * estimate to zero; the gate drops that row and the pair-gyro channel keeps observing the joint.
    */
   @Test
   public void testAStuckZeroVelocityDoesNotCollapseTheEstimate()
   {
      double estimated = estimatedVelocityAfterRun(directVelocityFixture(1L, 4, 1, 2), TRUE_QD, 0.0);

      assertTrue(estimated > 0.5 * TRUE_QD,
                 "a q̇ frozen at 0 pulled the estimate to " + estimated + " rad/s against a true " + TRUE_QD
                 + " rad/s — the stuck-velocity gate is not holding");
   }

   /** A frozen NON-zero reading is the same failure; the gate must not be special-cased to zero. */
   @Test
   public void testAStuckNonZeroVelocityIsAlsoRejected()
   {
      double wrong = -TRUE_QD;
      double estimated = estimatedVelocityAfterRun(directVelocityFixture(2L, 4, 1, 2), TRUE_QD, wrong);

      assertTrue(estimated > 0.0,
                 "a q̇ frozen at " + wrong + " dragged the estimate to " + estimated + " — it should not even "
                 + "have kept the sign of a reading the encoder contradicts");
   }

   /**
    * The other half of the contract: at true rest the reading is legitimately constant, the encoder agrees,
    * and the row must survive. A staleness rule that looked only at q̇ would fail this and quietly switch the
    * channel off during stance.
    */
   @Test
   public void testAJointGenuinelyAtRestKeepsItsVelocityRow()
   {
      JointLevelKFTestFixture fixture = directVelocityFixture(3L, 4, 1, 2);
      double estimated = estimatedVelocityAfterRun(fixture, 0.0, 0.0);

      assertEquals(0.0, estimated, 0.02, "a resting joint's velocity must stay at zero");
      for (OneDoFJointBasics joint : fixture.filteredJoints)
         assertTrue(fixture.filter.isDirectVelocityRowUsedForTest(joint),
                    joint.getName() + "'s velocity row was dropped at rest — the gate is reading a constant "
                    + "measurement as stuck without checking the encoder, which disables the channel exactly "
                    + "where it is supposed to help");
   }

   /**
    * A dropout on ONE joint must not drop the others' rows. This is the all-or-nothing bug that was already
    * fixed for encoder positions, on hardware whose encoders are documented as intermittent.
    */
   @Test
   public void testOneJointsDropoutDoesNotDisableTheOthersVelocityRows()
   {
      // A wider IMU-pair span so the filter takes on several joints: the single-joint chain the other tests
      // use cannot distinguish a per-joint gate from an all-or-nothing one.
      JointLevelKFTestFixture fixture = directVelocityFixture(4L, 6, 1, 4);
      List<OneDoFJointBasics> joints = fixture.filteredJoints;
      int n = joints.size();
      assertTrue(n > 1, "this test needs at least two filtered joints to distinguish per-joint from all-or-nothing");

      double[] qTrue = new double[n];
      double[] qdTrue = new double[n];
      for (int i = 0; i < n; i++)
         qdTrue[i] = TRUE_QD;

      List<RigidBodyBasics> noTrustedFeet = new ArrayList<>();
      for (int tick = 0; tick < TICKS; tick++)
      {
         for (int i = 0; i < n; i++)
            qTrue[i] += qdTrue[i] * JointLevelKFTestFixture.DT;
         fixture.applyConsistentMotion(qTrue, qdTrue);
         for (int i = 0; i < n; i++)
            fixture.setMeasuredVelocity(joints.get(i), i == 0 ? Double.NaN : TRUE_QD);
         fixture.filter.computeJointState();
         fixture.filter.computeImuBiases(noTrustedFeet);
      }

      assertTrue(fixture.filter.isDirectVelocityRowUsedForTest(joints.get(1)),
                 "joint 1's velocity row was dropped because joint 0 had a dropout — the channel is still "
                 + "all-or-nothing");
      assertEquals(TRUE_QD, fixture.filter.getEstimatedJointVelocity(joints.get(1)), 0.05,
                   "a healthy joint must keep tracking while another joint drops out");
   }

   /** Setting the hold to zero must switch the stuck gate off entirely, leaving the pre-gate behavior. */
   @Test
   public void testTheStuckGateCanBeDisabledLive()
   {
      JointLevelKFTestFixture fixture = directVelocityFixture(5L, 4, 1, 2);
      fixture.filter.setQdStaleHoldSecondsForTest(0.0);
      double estimated = estimatedVelocityAfterRun(fixture, TRUE_QD, 0.0);

      assertNotEquals(0.0, TRUE_QD, "fixture sanity");
      assertTrue(Math.abs(estimated) < 0.5 * TRUE_QD,
                 "with the gate disabled the stuck reading should dominate again (estimated " + estimated
                 + "); if it does not, this test is no longer exercising the gate at all");
   }
}
