package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Pins the indexing algebra that ties {@code JointKFState}'s Trove inverse maps to the {@code jointsByIndex} /
 * {@code imusByOrdinal} arrays that define the state order.
 *
 * <p>Before the Trove conversion the joint index was carried by a {@code LinkedHashMap} whose iteration order
 * happened to equal its own values, because the only writer was {@code putIfAbsent(j, map.size())}. Several
 * consumers silently leaned on that coincidence — most dangerously the encoder and direct-velocity measurement
 * fills, which laid row {@code r} by iteration counter while {@code H = I} and {@code R}'s diagonal were keyed
 * by state index. A permutation there routes every joint's measurement into another joint's state with no
 * residual blow-up and NIS still ~1: the filter fuses garbage quietly. Nothing in the suite caught it.</p>
 *
 * <p><b>Design rule for this file:</b> every assertion compares a quantity derived from the Trove <i>map</i>
 * against one derived from the <i>array</i>. Comparing two array-derived quantities is tautological now that
 * both sides share a source, and would pass no matter how badly the map disagreed. Note in particular that
 * {@code JointLevelKFEncoderNISConsistencyTest} and {@code JointLevelKFDirectVelocityMeasurementTest} DID
 * detect a permutation before the conversion (one side came from {@code keySet()}, the other from
 * {@code e.getValue()}) and are structurally self-consistent afterwards — which is precisely why these exist.</p>
 *
 * @author Lucas Libshutz
 */
public class JointLevelKFStateOrderPropertyTest
{
   /**
    * The load-bearing invariant: the published state order and the map that resolves a joint to its state index
    * are exact inverses of each other. Everything else in this file is a specialization of this.
    */
   @Test
   public void stateIndexIsTheExactInverseOfTheStateOrderList()
   {
      List<JointLevelKFTestFixture> fixtures = new ArrayList<>(JointLevelKFTestFixture.shapes(9100L));
      fixtures.add(JointLevelKFTestFixture.twoPairs(9200L, 10, 1, 5, 9));
      fixtures.add(JointLevelKFTestFixture.singlePairFootBeyondIMUs(9300L, 10, 1, 7, 9));

      for (JointLevelKFTestFixture f : fixtures)
      {
         List<OneDoFJointBasics> stateOrder = f.filter.getFilteredJointsInStateOrder();
         assertEquals(f.n, stateOrder.size(), f.describe() + " state-order list length must equal n");

         for (int i = 0; i < f.n; i++)
            assertEquals(i,
                         f.filter.getJointStateIndex(stateOrder.get(i)),
                         f.describe() + " joint at state-order position " + i + " must report state index " + i);

         // Bijection: no joint may occupy two slots, which a map/array disagreement could otherwise hide.
         for (int i = 0; i < f.n; i++)
            for (int j = i + 1; j < f.n; j++)
               assertNotEquals(stateOrder.get(i),
                               stateOrder.get(j),
                               f.describe() + " joint repeated at state-order positions " + i + " and " + j);
      }
   }

   /**
    * The Trove-specific regression test. {@code TObjectIntHashMap}'s DEFAULT no-entry value is 0, which is a
    * perfectly valid joint state index — so a map built without the explicit -1 sentinel reports "state index
    * 0" for a joint that is not in the filter at all. This fails the day someone drops that constructor
    * argument, and it is the reason {@code JointKFState.NOT_IN_STATE} exists.
    */
   @Test
   public void unfilteredJointReportsNotInStateNotZero()
   {
      // The foot joint sits beyond both IMUs, so it is an anchor chain joint that is NOT a filter state.
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePairFootBeyondIMUs(9400L, 10, 1, 7, 9);
      List<OneDoFJointBasics> stateOrder = f.filter.getFilteredJointsInStateOrder();

      int unfilteredSeen = 0;
      for (OneDoFJointBasics joint : f.joints)
      {
         if (stateOrder.contains(joint))
            continue;
         unfilteredSeen++;
         assertEquals(JointKFState.NOT_IN_STATE,
                      f.filter.getJointStateIndex(joint),
                      f.describe() + " unfiltered joint " + joint.getName() + " must report NOT_IN_STATE, not a valid index");
         assertFalse(f.filter.containsJoint(joint), f.describe() + " unfiltered joint " + joint.getName() + " must not be reported as filtered");
      }
      assertTrue(unfilteredSeen > 0, f.describe() + " fixture was supposed to leave at least one joint out of the filter state");
   }

   /**
    * Encoder measurement row {@code i} must be the joint at state index {@code i}. Henc is the identity and
    * Renc's diagonal is keyed by state index, so any disagreement here fuses each encoder into the wrong joint.
    */
   @Test
   public void encoderMeasurementRowMatchesStateIndex()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(9500L))
      {
         // The encoder value is keyed to the JOINT, and the row it is expected on is resolved through the Trove
         // map — never by position in the state-order list. Reading both from the list would make this
         // tautological: the list and the fill loop share jointsByIndex, so they permute together.
         List<OneDoFJointBasics> filtered = f.filter.getFilteredJointsInStateOrder();
         for (OneDoFJointBasics joint : filtered)
            f.setEncoder(joint, encoderValueFor(joint));

         f.filter.initialize();
         f.filter.computeJointState();

         DMatrixRMaj zEnc = f.filter.getEncoderMeasurementForTest();
         for (OneDoFJointBasics joint : filtered)
         {
            int idx = f.filter.getJointStateIndex(joint);
            assertEquals(encoderValueFor(joint),
                         zEnc.get(idx, 0),
                         0.0,
                         f.describe() + " z_enc row " + idx + " must carry the encoder of " + joint.getName());
         }
      }
   }

   /** Same identity for the direct-velocity channel: z_qd row i is the joint at state index i. */
   @Test
   public void directVelocityMeasurementRowMatchesStateIndex()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePairWithDirectVelocity(9600L,
                                                                                       10,
                                                                                       1,
                                                                                       9,
                                                                                       name -> 1.0e-3,
                                                                                       name -> 1.0e-2,
                                                                                       name -> 20.0);
      List<OneDoFJointBasics> filtered = f.filter.getFilteredJointsInStateOrder();
      for (OneDoFJointBasics joint : filtered)
         f.setMeasuredVelocity(joint, velocityValueFor(joint));

      f.filter.initialize();
      f.filter.computeJointState();

      DMatrixRMaj zqd = f.filter.getDirectVelocityMeasurementForTest();
      for (OneDoFJointBasics joint : filtered)
      {
         int idx = f.filter.getJointStateIndex(joint); // map-derived row, not list position
         assertEquals(velocityValueFor(joint),
                      zqd.get(idx, 0),
                      0.0,
                      f.describe() + " z_qd row " + idx + " must carry the measured velocity of " + joint.getName());
      }
   }

   /**
    * The per-joint YoVariables are created in one loop and written in another. If those two disagree, SCS shows
    * every joint's estimate under a neighbour's name — invisible to any test that only checks matrices.
    */
   @Test
   public void publishedPerJointYoVariablesFollowStateIndex()
   {
      for (JointLevelKFTestFixture f : JointLevelKFTestFixture.shapes(9700L))
      {
         List<OneDoFJointBasics> filtered = f.filter.getFilteredJointsInStateOrder();
         for (OneDoFJointBasics joint : filtered)
            f.setEncoder(joint, encoderValueFor(joint));

         f.filter.initialize();
         f.filter.computeJointState();

         DMatrixRMaj x = f.filter.getStateVector();
         for (OneDoFJointBasics joint : filtered)
         {
            String jointName = joint.getName();
            int idx = f.filter.getJointStateIndex(joint); // map-derived, so a permuted array is detectable
            YoDouble q = (YoDouble) f.registry.findVariable("jointKF_q_" + jointName);
            YoDouble qd = (YoDouble) f.registry.findVariable("jointKF_qd_" + jointName);
            assertTrue(q != null && qd != null, f.describe() + " missing published YoVariables for " + jointName);
            assertEquals(x.get(idx), q.getValue(), 0.0, f.describe() + " jointKF_q_" + jointName + " must publish x[" + idx + "]");
            assertEquals(x.get(f.n + idx),
                         qd.getValue(),
                         0.0,
                         f.describe() + " jointKF_qd_" + jointName + " must publish x[" + (f.n + idx) + "]");
         }
      }
   }

   /**
    * Encoder variance is resolved by joint NAME into an array indexed by STATE index. A permutation between
    * those two gives every joint its neighbour's measurement noise — a silent mis-weighting, not a crash.
    */
   @Test
   public void perJointEncoderNoiseFollowsStateIndex()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePairWithEncoderNoise(9800L, 10, 1, 9, JointLevelKFStateOrderPropertyTest::uniqueSigmaForName);
      List<OneDoFJointBasics> filtered = f.filter.getFilteredJointsInStateOrder();
      f.filter.initialize();

      DMatrixRMaj rEnc = f.filter.getEncoderNoise();
      for (OneDoFJointBasics joint : filtered)
      {
         int idx = f.filter.getJointStateIndex(joint); // map-derived row against name-derived variance
         double sigma = uniqueSigmaForName(joint.getName());
         assertEquals(sigma * sigma,
                      rEnc.get(idx, idx),
                      1.0e-18,
                      f.describe() + " Renc(" + idx + "," + idx + ") must be the wired variance of " + joint.getName());
      }
   }

   /**
    * IMU-side analogue of {@link #stateIndexIsTheExactInverseOfTheStateOrderList}: ordinals form a bijection
    * onto [0, m) and the bias block column is exactly 2n + 3o. A wrong ordinal here aims the gyro-bias export
    * at the wrong three columns, which under the -1 sentinel can land inside the q̇ block and stay in range.
    */
   @Test
   public void imuOrdinalIsTheInverseOfTheBiasBlockLayout()
   {
      List<JointLevelKFTestFixture> fixtures = new ArrayList<>(JointLevelKFTestFixture.shapes(9900L));
      fixtures.add(JointLevelKFTestFixture.twoPairs(9950L, 10, 1, 5, 9));

      for (JointLevelKFTestFixture f : fixtures)
      {
         boolean[] ordinalSeen = new boolean[f.m];
         for (JointLevelKFTestFixture.TestIMU imu : f.imus)
         {
            int ordinal = f.filter.getImuOrdinal(imu); // map-derived
            assertTrue(ordinal >= 0 && ordinal < f.m, f.describe() + " ordinal of " + imu.getSensorName() + " out of range: " + ordinal);
            assertFalse(ordinalSeen[ordinal], f.describe() + " ordinal " + ordinal + " assigned to two IMUs");
            ordinalSeen[ordinal] = true;

            // The inverse leg: the ORDINAL-INDEXED array must hand back the same IMU the map sent us to.
            // Without this the assertions here would be map-against-map and blind to a permuted array.
            assertEquals(imu,
                         f.filter.getImuByOrdinal(ordinal),
                         f.describe() + " imusByOrdinal[" + ordinal + "] must be " + imu.getSensorName());

            assertEquals(2 * f.n + 3 * ordinal,
                         f.filter.getBiasBlockColumn(imu),
                         f.describe() + " bias block column of " + imu.getSensorName() + " must be 2n + 3*ordinal");
         }
         for (int o = 0; o < f.m; o++)
            assertTrue(ordinalSeen[o], f.describe() + " no IMU claimed ordinal " + o);
      }
   }

   // Per-JOINT values, keyed by identity rather than by state position. That is what lets the assertions above
   // resolve the expected row through the Trove map and still know what value belongs there.

   private static double encoderValueFor(OneDoFJointBasics joint)
   {
      return 1.0 + 0.37 * (1 + Math.floorMod(joint.getName().hashCode(), 89));
   }

   private static double velocityValueFor(OneDoFJointBasics joint)
   {
      return -2.0 - 0.11 * (1 + Math.floorMod(joint.getName().hashCode(), 83));
   }

   /** Distinct, deterministic per-joint encoder sigma so a permuted wiring cannot coincidentally match. */
   private static double uniqueSigmaForName(String jointName)
   {
      return 1.0e-4 * (1 + Math.floorMod(jointName.hashCode(), 97));
   }
}
