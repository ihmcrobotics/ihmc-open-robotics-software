package us.ihmc.stateEstimation.jointLevel;

import gnu.trove.impl.Constants;
import gnu.trove.map.hash.TObjectIntHashMap;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUBasedJointStateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.function.ToDoubleFunction;

/**
 * The joint-level KF's state layout and storage, plus the structural holders ({@link Pair}, {@link FootAnchor})
 * every other component reads. Layout — none of it may be reordered:
 * <pre>
 *   x = [ q(0..n-1) ; q̇(n..2n-1) ; b_omega(2n..2n+3m-1) ] ,  dim = 2n + 3m
 * </pre>
 * Joint state index and IMU ordinal are the POSITIONS in {@link #jointsByIndex} / {@link #imusByOrdinal},
 * assigned in first-encounter order as the IMU-pair chains are resolved. Those two arrays ARE the state order.
 * {@code jointToIndex} / {@code imuToOrdinal} are the inverse lookup ONLY and are private for one reason: they
 * are hash maps, so their iteration order is bucket order and has nothing to do with the state layout, and
 * every iterator they hand out allocates. Loop over joints and IMUs by index, never over a map.
 * IMU {@code o}'s bias occupies columns {@code [2n+3o, 2n+3o+3)}; the stacked gyro measurement lays pair
 * {@code e} on rows {@code [3e, 3e+3)} with the active anchors trailing.
 *
 * <p>Fields are package-private and read directly rather than through getters, so the hot path stays
 * allocation-free and monomorphic.</p>
 *
 * @author Lucas Libshutz
 */
final class JointKFState
{
   private final JointKFParameters parameters;

   final SensorOutputMapReadOnly sensorMap;
   final double dt;

   /**
    * Returned by {@link #jointIndex} / {@link #imuOrdinal} for a key that is NOT part of the filter state.
    * Trove's DEFAULT no-entry value is 0, which is a valid joint index and a valid IMU ordinal — the -1 passed
    * to both constructors below is therefore mandatory, not stylistic, and every lookup must test against it.
    */
   static final int NOT_IN_STATE = -1;

   /**
    * Inverse lookup ONLY, and private so nothing can iterate them (see the class javadoc) or write to them
    * after construction. Deliberately NOT {@code JointIndexHandler}: that stores a
    * {@code LinkedHashMap<JointReadOnly, int[]>}, so it is neither unboxed nor iteration-free; it hands back
    * full-multibody DoF column blocks rather than this filter's dense 0..n-1 state indices; and it has no
    * IMU-ordinal equivalent at all. {@link #jointsByIndex} IS this filter's index handler.
    */
   private final TObjectIntHashMap<OneDoFJointBasics> jointToIndex;
   private final TObjectIntHashMap<IMUSensorReadOnly> imuToOrdinal;
   final List<Pair> pairs = new ArrayList<>();
   final List<FootAnchor> footAnchors = new ArrayList<>();
   /** state index -> joint. THE definition of state order: x[i] is the position of {@code jointsByIndex[i]}. */
   OneDoFJointBasics[] jointsByIndex;
   /** ordinal -> IMU, so a per-tick loop over IMUs is an index loop (no map-iterator allocation). */
   IMUSensorReadOnly[] imusByOrdinal;
   IMUSensorReadOnly baseIMU;

   int baseBiasCol;
   int numberOfJoints;   // number of distinct joints
   int numberOfIMUs;   // number of IMUs
   int dim; // 2n + 3m
   int numberOfIMUPairs;   // number of IMU pairs (fixed at construction)
   /** 3 * (E + K_max), K_max = number of foot anchors: the widest stacked measurement. */
   int maxStackRows;

   /** x = [q ; q_dot ; b_omega]. */
   final DMatrixRMaj x = new DMatrixRMaj(0, 1);
   final DMatrixRMaj P = new DMatrixRMaj(0, 0);
   boolean initialized = false;

   /**
    * ONE flag across every non-finite-input site (seeding, the encoder/velocity scans, the unfiltered anchor
    * joints, the Joseph rollback), deliberately shared so the FIRST offender in the boot sequence is the one
    * named. The singular-innovation diagnostic keeps its own flag so the two do not swallow each other.
    */
   boolean warnedNonFiniteInput = false;
   private boolean nonFiniteStateReported = false;

   private final YoInteger yoStateDimension;
   private final YoInteger yoNumberOfFilteredJoints;
   private final YoInteger yoNumberOfIMUs;

   // Per-joint (q, qd) and the +/- 1 sigma envelope around each, indexed by joint state index. Allocated once,
   // then only .set() on the estimator thread.
   private YoDouble[] yoJointPosition;
   private YoDouble[] yoJointVelocity;
   private YoDouble[] yoJointPositionUpperBound;
   private YoDouble[] yoJointPositionLowerBound;
   private YoDouble[] yoJointVelocityUpperBound;
   private YoDouble[] yoJointVelocityLowerBound;

   /**
    * @param encoderVelocityNoiseStd consumed ONLY for {@link FootAnchor#qdVar}, which is resolved in the same
    *                 anchor-chain walk that classifies each leg joint as filtered or unfiltered — doing it in
    *                 {@link JointKFUpdate} would mean walking every chain twice.
    */
   JointKFState(SensorOutputMapReadOnly sensorMap,
                List<IMUBasedJointStateEstimatorParameters> pairParameters,
                Collection<RigidBodyBasics> feet,
                ToDoubleFunction<String> encoderVelocityNoiseStd,
                double estimatorDT,
                JointKFParameters parameters,
                YoRegistry registry)
   {
      this.sensorMap = sensorMap;
      this.dt = estimatorDT;
      this.parameters = parameters;

      // Capacities only; both maps are filled exactly once, in the pass below, and never written again. The IMU
      // count is bounded exactly at 2 per pair; the joint count is not known until the chains are resolved, so
      // this is a generous guess whose only cost is one boot-time rehash. NOT_IN_STATE is the no-entry value.
      jointToIndex = new TObjectIntHashMap<>(64, Constants.DEFAULT_LOAD_FACTOR, NOT_IN_STATE);
      imuToOrdinal = new TObjectIntHashMap<>(Math.max(1, 2 * pairParameters.size()), Constants.DEFAULT_LOAD_FACTOR, NOT_IN_STATE);

      // 1)  Resolve all pairs of IMUs, collect distinct joints and IMUs into the state layout
      for (IMUBasedJointStateEstimatorParameters pp : pairParameters)
      {
         IMUSensorReadOnly parent = findIMU(sensorMap, pp.getParentIMUName());
         IMUSensorReadOnly child = findIMU(sensorMap, pp.getChildIMUName());
         if (parent == null || child == null)
         {
            LogTools.warn("Skipping pair, IMU not found: parent = " + pp.getParentIMUName() + " child = " + pp.getChildIMUName());
            continue;
         }
         // Structural asserts (Part B item 6): a self-pair or a pair whose two IMUs sit on the same measurement
         // link has zero H, zero z and zero noise — S is singular by construction (SPEC §5). Fail loud at boot.
         if (parent == child)
            throw new IllegalArgumentException("JointLevelKFPreFilter: pair '" + pp.getEstimatorName() + "' has the same IMU ("
                  + parent.getSensorName() + ") as parent and child.");
         if (parent.getMeasurementLink() == child.getMeasurementLink())
            throw new IllegalArgumentException("JointLevelKFPreFilter: pair '" + pp.getEstimatorName() + "' parent and child IMUs "
                  + "share the measurement link " + parent.getMeasurementLink().getName() + " (zero-DoF chain).");
         Pair pair = new Pair(parent, child);
         pair.jacobian.setKinematicChain(parent.getMeasurementLink(), child.getMeasurementLink());
         pair.jacobian.setJacobianFrame(child.getMeasurementLink().getBodyFixedFrame());
         //NOTE: Assumes 1-DoF and fixed joints only, just as IMUBasedJointStateEstimator does.
         // The i-th angular jacobian column correspnods to the i-th filtered joint.
         List<OneDoFJointBasics> chain = MultiBodySystemTools.filterJoints(pair.jacobian.getJointsFromBaseToEndEffector(), OneDoFJointBasics.class);
         pair.chainJoints = chain.toArray(new OneDoFJointBasics[0]);
         for (OneDoFJointBasics j : pair.chainJoints)
            jointToIndex.putIfAbsent(j, jointToIndex.size()); // places joints via hash mapping
         imuToOrdinal.putIfAbsent(parent, imuToOrdinal.size());
         imuToOrdinal.putIfAbsent(child, imuToOrdinal.size());
         pairs.add(pair);
      }

      numberOfJoints = jointToIndex.size();
      numberOfIMUs = imuToOrdinal.size();
      dim = 2 * numberOfJoints + 3 * numberOfIMUs;

      // index -> key inverses, built ONCE, here, before anything reads them. Built by re-walking `pairs` — the
      // same source that populated the maps — so not even construction has to iterate a hash map. Writes are
      // idempotent for a joint that appears on two chains.
      jointsByIndex = new OneDoFJointBasics[numberOfJoints];
      imusByOrdinal = new IMUSensorReadOnly[numberOfIMUs];
      for (Pair pair : pairs)
      {
         for (OneDoFJointBasics j : pair.chainJoints)
            jointsByIndex[jointToIndex.get(j)] = j;
         imusByOrdinal[imuToOrdinal.get(pair.parent)] = pair.parent;
         imusByOrdinal[imuToOrdinal.get(pair.child)] = pair.child;
      }

      // Acyclicity assert (Part B item 6): with the exact R_g, a cycle's telescoping row-combination has zero H,
      // zero z and zero noise, so S is singular BY CONSTRUCTION (SPEC §5.3). The used IMU graph MUST be a tree.
      assertAcyclicIMUGraph();

      // 2) Second pass: assemble state sinze columns are known as n is fixed.
      for (Pair pair : pairs)
      {
         int numberOfChainDoFs = pair.chainJoints.length;
         pair.Jang.reshape(3, numberOfChainDoFs); // Jang is allocated blank at construction in the chain, only reshaped once full DoF are known
         pair.qdCols = new int[numberOfChainDoFs];
         for (int c = 0; c < numberOfChainDoFs; c++)
         {
            // Fail loud rather than let a sentinel through: numberOfJoints + (-1) is an IN-RANGE column, so a
            // missing chain joint would silently alias another joint's q̇ instead of throwing. Unreachable —
            // the first pass inserted every chain joint of every pair.
            int stateIndex = jointToIndex.get(pair.chainJoints[c]);
            if (stateIndex == NOT_IN_STATE)
               throw new IllegalStateException("JointLevelKFPreFilter: chain joint " + pair.chainJoints[c].getName() + " of pair "
                     + pair.parent.getSensorName() + "->" + pair.child.getSensorName() + " is not a filter state.");
            pair.qdCols[c] = numberOfJoints + stateIndex; // this int[] is the selector matrix of the path S_ab, but in sparse form as all joints are not related directly
         }
         pair.parentBias = 2 * numberOfJoints + 3 * requireImuOrdinal(pair.parent);
         pair.childBias = 2 * numberOfJoints + 3 * requireImuOrdinal(pair.child);
      }

      // 3) Base IMU = root of the tree + first pair parent.
      //WARNING: Need to configure if the root differs.
      baseIMU = pairs.isEmpty() ? null : pairs.get(0).parent;
      baseBiasCol = baseIMU == null ? -1 : 2 * numberOfJoints + 3 * requireImuOrdinal(baseIMU);
      if (baseIMU == null)
         throw new RuntimeException("Base IMU is null, check the kinematic tree.");
      LogTools.info("Base IMU initialized as " + baseIMU.getSensorName());

      // 4) Precompute base->foot chains for the stance anchoring measurement update
      if (feet != null)
      {
         double sigmaQdUnfiltered = parameters.sigmaQdUnfiltered.getValue();
         double qdVarFallback = sigmaQdUnfiltered * sigmaQdUnfiltered;
         for (RigidBodyBasics foot : feet)
         {
            FootAnchor footAnchor = new FootAnchor(foot);
            footAnchor.jacobian.setKinematicChain(baseIMU.getMeasurementLink(), foot);
            footAnchor.jacobian.setJacobianFrame(baseIMU.getMeasurementFrame()); // express J in the same frame as the base gyro
            List<OneDoFJointBasics> leg = MultiBodySystemTools.filterJoints(footAnchor.jacobian.getJointsFromBaseToEndEffector(), OneDoFJointBasics.class);

            footAnchor.legJoints = leg.toArray(new OneDoFJointBasics[0]);
            footAnchor.Jang.reshape(3, footAnchor.legJoints.length);
            footAnchor.qdCols = new int[footAnchor.legJoints.length];
            footAnchor.qdVar = new double[footAnchor.legJoints.length];

            // A leg joint that is NOT a filter state does not disable the anchor -- it only has to be KNOWN, not
            // ESTIMATED, so it moves to the measurement side (see JointKFBiasUpdate.buildStackedMeasurement).
            // The +I3 on b_base, the only thing fixing the common-mode bias gauge, is untouched by that split.
            // This used to set usable=false for ANY unfiltered chain joint, which on Alex is always (no foot
            // IMUs => ankles never filtered => both anchors dead every tick, bias gauge never fixed, pelvis gyro
            // bias random-walked to 0.17 rad/s and drove the pitch drift). See FINDINGS.md Part F.
            footAnchor.usable = true;
            int unfiltered = 0;
            for (int c = 0; c < footAnchor.legJoints.length; c++)
            {
               int stateIndex = jointToIndex.get(footAnchor.legJoints[c]);
               if (stateIndex == NOT_IN_STATE)
               {
                  footAnchor.qdCols[c] = -1; // unfiltered: contributes J_U * qd^meas to z', no H column
                  unfiltered++;
               }
               else
                  footAnchor.qdCols[c] = numberOfJoints + stateIndex;
               // Per-joint encoder VELOCITY variance for the anchor's input-noise congruence (only the
               // unfiltered columns are ever read, but fill every slot — cheap, and no -1 bookkeeping).
               double qdStd = encoderVelocityNoiseStd == null ? Double.NaN : encoderVelocityNoiseStd.applyAsDouble(footAnchor.legJoints[c].getName());
               footAnchor.qdVar[c] = (Double.isFinite(qdStd) && qdStd > 0.0) ? qdStd * qdStd : qdVarFallback;
            }
            // A chain with no joints at all cannot anchor anything (degenerate model).
            if (footAnchor.legJoints.length == 0)
               footAnchor.usable = false;
            if (unfiltered > 0)
               LogTools.info("JointLevelKFPreFilter: foot anchor " + foot.getName() + " has " + unfiltered + " of "
                             + footAnchor.legJoints.length + " chain joints unfiltered; their measured velocities are folded into the "
                             + "anchor measurement and their encoder noise into the anchor covariance.");
            footAnchors.add(footAnchor);
         }
      }

      // Structural assert (gauge fixing): with no usable anchor the base-IMU gyro bias is unobservable FOREVER
      // -- it random-walks and integrates straight into base orientation. Not a degraded mode, a broken filter,
      // and it must never boot silently again (it did, for months). Fail loud.
      if (feet != null && !feet.isEmpty() && footAnchors.stream().noneMatch(footAnchor -> footAnchor.usable))
         throw new IllegalArgumentException("JointLevelKFPreFilter: no usable foot anchor. The base-IMU gyro bias is a gauge "
               + "freedom fixed ONLY by the stance-anchor rows; without one it is unobservable and will diverge. "
               + "Check that the base IMU's measurement link connects to at least one foot.");

      // Layout sizes the measurement components size themselves against. Computed HERE, once, so nothing has to
      // reach into a component constructed later.
      numberOfIMUPairs = pairs.size();
      maxStackRows = 3 * (numberOfIMUPairs + footAnchors.size());

      x.reshape(dim, 1);
      P.reshape(dim, dim);

      yoStateDimension = new YoInteger("jointKFStateDimension", registry);
      yoNumberOfFilteredJoints = new YoInteger("jointKFNumberOfFilteredJoints", registry);
      yoNumberOfIMUs = new YoInteger("jointKFNumberOfIMUs", registry);
      yoStateDimension.set(dim);
      yoNumberOfFilteredJoints.set(numberOfJoints);
      yoNumberOfIMUs.set(numberOfIMUs);
      createJointYoVariables(registry);
   }

   /** Filter state index of {@code joint}'s position entry (its velocity entry is this + n), or {@link #NOT_IN_STATE}. */
   int jointIndex(OneDoFJointBasics joint)
   {
      return jointToIndex.get(joint);
   }

   /** Whether {@code joint} is an estimated filter state. Takes the supertype for the spanning-joint walk. */
   boolean isFilteredJoint(JointReadOnly joint)
   {
      return jointToIndex.containsKey(joint);
   }

   /** Bias-block ordinal of {@code imu}, or {@link #NOT_IN_STATE}. */
   int imuOrdinal(IMUSensorReadOnly imu)
   {
      return imuToOrdinal.get(imu);
   }

   /**
    * Ordinal of an IMU that MUST be in the state. Fails loud rather than returning a -1 that would index three
    * columns backwards out of the bias block and into the q̇ block — an in-range, silently wrong answer.
    */
   int requireImuOrdinal(IMUSensorReadOnly imu)
   {
      int ordinal = imuToOrdinal.get(imu);
      if (ordinal == NOT_IN_STATE)
         throw new IllegalArgumentException("JointLevelKFPreFilter: IMU '" + imu.getSensorName() + "' is not part of the filter state.");
      return ordinal;
   }

   /** One set per filtered joint, indexed by state index so the per-tick update is a straight array write. */
   private void createJointYoVariables(YoRegistry registry)
   {
      yoJointPosition = new YoDouble[numberOfJoints];
      yoJointVelocity = new YoDouble[numberOfJoints];
      yoJointPositionUpperBound = new YoDouble[numberOfJoints];
      yoJointPositionLowerBound = new YoDouble[numberOfJoints];
      yoJointVelocityUpperBound = new YoDouble[numberOfJoints];
      yoJointVelocityLowerBound = new YoDouble[numberOfJoints];
      for (int stateIndex = 0; stateIndex < numberOfJoints; stateIndex++)
      {
         String jointName = jointsByIndex[stateIndex].getName();
         yoJointPosition[stateIndex] = new YoDouble("jointKF_q_" + jointName, registry);
         yoJointVelocity[stateIndex] = new YoDouble("jointKF_qd_" + jointName, registry);
         yoJointPositionUpperBound[stateIndex] = new YoDouble("jointKF_q_" + jointName + "_upperBound", registry);
         yoJointPositionLowerBound[stateIndex] = new YoDouble("jointKF_q_" + jointName + "_lowerBound", registry);
         yoJointVelocityUpperBound[stateIndex] = new YoDouble("jointKF_qd_" + jointName + "_upperBound", registry);
         yoJointVelocityLowerBound[stateIndex] = new YoDouble("jointKF_qd_" + jointName + "_lowerBound", registry);
      }
   }

   /**
    * Seeds x and P from the current encoders and marks the filter initialized. Returns false (to be retried
    * next tick) if any encoder reads non-finite: a KF permanently latches NaN, so bad boot data must not be
    * latched. While uninitialized, consumers see NaN/zero and fall back to the raw sensors.
    */
   boolean seed()
   {
      // Index loops, not map iteration: seed() runs EVERY tick until the encoders come good, so an entry-set
      // iterator here allocates on the estimator thread for the whole boot window. The allocation test never
      // sees it because it calls initialize() before it starts measuring.
      for (int i = 0; i < numberOfJoints; i++)
      {
         if (!Double.isFinite(sensorMap.getOneDoFJointOutput(jointsByIndex[i]).getPosition()))
         {
            warnNonFiniteInputOnce("joint position of " + jointsByIndex[i].getName() + " at initialization");
            return false;
         }
      }
      x.zero();
      for (int i = 0; i < numberOfJoints; i++)
         x.set(i, sensorMap.getOneDoFJointOutput(jointsByIndex[i]).getPosition());
      P.zero();
      for (int i = 0; i < numberOfJoints; i++) P.set(i, i, parameters.initPosVar.getValue());
      for (int i = numberOfJoints; i < 2 * numberOfJoints; i++) P.set(i, i, parameters.initVelVar.getValue());
      for (int i = 2 * numberOfJoints; i < dim; i++) P.set(i, i, parameters.initBiasVar.getValue());
      initialized = true;
      return true;
   }

   /**
    * Publishes (q, qd) and their 1-sigma envelope straight from x and P; allocation-free. The variance is
    * clamped at 0 before the sqrt, to stay finite through the numerical negatives P can momentarily take.
    */
   void updateJointYoVariables()
   {
      for (int i = 0; i < numberOfJoints; i++)
      {
         double q = x.get(i);
         double qd = x.get(numberOfJoints + i);
         double sigmaQ = Math.sqrt(Math.max(0.0, P.get(i, i)));
         double sigmaQd = Math.sqrt(Math.max(0.0, P.get(numberOfJoints + i, numberOfJoints + i)));
         yoJointPosition[i].set(q);
         yoJointVelocity[i].set(qd);
         yoJointPositionUpperBound[i].set(q + sigmaQ);
         yoJointPositionLowerBound[i].set(q - sigmaQ);
         yoJointVelocityUpperBound[i].set(qd + sigmaQd);
         yoJointVelocityLowerBound[i].set(qd - sigmaQd);
      }
   }

   /**
    * Logs the FIRST non-finite input ever seen, once — naming whichever sensor is late in the boot sequence —
    * then stays silent. Runs on the estimator thread, so the string concat happens only on that occurrence.
    */
   void warnNonFiniteInputOnce(String source)
   {
      if (warnedNonFiniteInput)
         return;
      warnedNonFiniteInput = true;
      LogTools.warn("Non-finite input to JointLevelKFPreFilter; first offender: " + source
            + ". Affected updates are skipped and consumers fall back to raw sensors / zero bias.");
   }

   /**
    * One-shot triage: the first time x or P goes non-finite, names the stage that produced it, then stays
    * quiet. With the input/inverse/rollback guards elsewhere this should never fire; if it does, that stage
    * name is the exact place NaN enters.
    */
   void warnIfNonFiniteState(String stage, int index)
   {
      if (nonFiniteStateReported)
         return;
      if (JointLevelKFPreFilter.containsNonFinite(x) || JointLevelKFPreFilter.containsNonFinite(P))
      {
         nonFiniteStateReported = true;
         LogTools.error("JointLevelKFPreFilter state first went non-finite after stage '" + stage + "'"
                        + (index >= 0 ? " #" + index : "") + " [x non-finite=" + JointLevelKFPreFilter.containsNonFinite(x)
                        + ", P non-finite=" + JointLevelKFPreFilter.containsNonFinite(P) + "]. This is where the NaN enters.");
      }
   }

   /**
    * Logs the chain-DoF census, and for each 1-DoF chain its joint axis and the two gyro axes orthogonal to it.
    * Those two rows observe PURE bias, so with a zero gyro Sigma their innovation variance has no floor — they
    * are the min-side rows the Sigma floor protects. Construction-only; never on the hot path.
    */
   void logChainDoFCensus()
   {
      StringBuilder message = new StringBuilder("JointLevelKFPreFilter chain-DoF census (" + pairs.size() + " pairs over " + numberOfIMUs + " IMUs, tree):");
      for (Pair pair : pairs)
      {
         int numberOfChainDoFs = pair.chainJoints.length;
         message.append("\n  ").append(pair.parent.getSensorName()).append("->").append(pair.child.getSensorName())
                .append(" : ").append(numberOfChainDoFs).append("-DoF chain [").append(jointNamesOf(pair.chainJoints)).append("]");
         if (numberOfChainDoFs == 1)
         {
            Vector3DReadOnly axis = pair.chainJoints[0].getJointAxis();
            message.append("  (1-DoF sentinel: joint axis ~[").append(String.format("%.2f,%.2f,%.2f", axis.getX(), axis.getY(), axis.getZ()))
                   .append("]; the two rows orthogonal to it observe PURE bias — min-side S floor = Sigma)");
         }
      }
      LogTools.info(message.toString());
   }

   /** Reverse of {@code jointToIndex} for the diagnostic paths: filter-joint name at a given state index. */
   String jointNameByStateIndex(int stateIndex)
   {
      return (stateIndex >= 0 && stateIndex < numberOfJoints) ? jointsByIndex[stateIndex].getName() : "?stateIndex" + stateIndex;
   }

   static String jointNamesOf(OneDoFJointBasics[] joints)
   {
      StringBuilder names = new StringBuilder();
      for (int i = 0; i < joints.length; i++)
      {
         if (i > 0)
            names.append(",");
         names.append(joints[i].getName());
      }
      return names.toString();
   }

   /**
    * Union-find acyclicity check on the used IMU graph. A cycle's telescoping row-combination carries the
    * identity 0 = 0 (zero H, zero z, zero noise), so with the exact R_g it makes the stacked S singular BY
    * CONSTRUCTION (SPEC §5.3). Throws naming the redundant pair to drop. Construction-only.
    */
   private void assertAcyclicIMUGraph()
   {
      int[] parent = new int[numberOfIMUs];
      for (int i = 0; i < numberOfIMUs; i++)
         parent[i] = i;
      for (Pair pair : pairs)
      {
         int a = find(parent, requireImuOrdinal(pair.parent));
         int b = find(parent, requireImuOrdinal(pair.child));
         if (a == b)
            throw new IllegalStateException("JointLevelKFPreFilter: the used IMU graph has a CYCLE — the pair "
                  + pair.parent.getSensorName() + "->" + pair.child.getSensorName() + " closes a loop. With the exact R_g a "
                  + "cycle's telescoping row-combination has zero H, zero z and zero noise, so the stacked innovation "
                  + "covariance S is singular by construction (SPEC §5.3). Drop this redundant pair — its chain joints "
                  + "are already covered by the remaining edges, so removing it loses no information.");
         parent[a] = b;
      }
      // E <= m - 1 for a forest; with connectivity from the base IMU the used graph is a tree. (A disconnected
      // forest is not an error here — an IMU island simply contributes an independent sub-filter.)
   }

   private static int find(int[] parent, int i)
   {
      while (parent[i] != i)
      {
         parent[i] = parent[parent[i]];
         i = parent[i];
      }
      return i;
   }

   private static IMUSensorReadOnly findIMU(SensorOutputMapReadOnly map, String name)
   {
      for (int i = 0; i < map.getIMUOutputs().size(); i++)
      {
         IMUSensorReadOnly s = map.getIMUOutputs().get(i);
         if (s.getSensorName().equals(name)) return s;
      }
      return null;
   }

   // ================================ Structure holders ================================
   // Shared, not private state: built here, but FootAnchor.active / .R are written and .qdVar read by
   // JointKFBiasUpdate.buildStackedMeasurement.

   static final class Pair
   {
      final IMUSensorReadOnly parent, child;
      final GeometricJacobianCalculator jacobian = new GeometricJacobianCalculator();
      OneDoFJointBasics[] chainJoints;
      int[] qdCols;
      int parentBias, childBias;
      final DMatrixRMaj Jang = new DMatrixRMaj(3, 1);

      Pair(IMUSensorReadOnly parent, IMUSensorReadOnly child)
      {
         this.parent = parent;
         this.child = child;
      }
   }

   static final class FootAnchor
   {
      final RigidBodyBasics foot;
      final GeometricJacobianCalculator jacobian = new GeometricJacobianCalculator();
      OneDoFJointBasics[] legJoints;
      /** Filter qd column per chain joint, or -1 if that joint is NOT a filter state (folded into z' instead). */
      int[] qdCols;
      /** Encoder velocity variance per chain joint ((rad/s)^2), consumed only for the unfiltered (-1) columns. */
      double[] qdVar;
      boolean usable; // false only for a degenerate (jointless) base->foot chain
      boolean active; // true this tick: usable AND trusted last tick (SPEC §6); set in buildStackedMeasurement
      final DMatrixRMaj Jang = new DMatrixRMaj(3, 1);
      /** Per-tick anchor covariance Sigma_eps + J_U Sigma_qdU J_U^T (configuration-dependent). */
      final DMatrixRMaj R = new DMatrixRMaj(3, 3);

      FootAnchor(RigidBodyBasics foot)
      {
         this.foot = foot;
      }
   }
}
