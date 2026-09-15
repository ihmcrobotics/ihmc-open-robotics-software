package us.ihmc.stateEstimation.invariantEstimator;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator.MomentumStateUpdater;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Center-of-mass state source for the invariant main estimator, derived <em>purely</em> from the InEKF
 * base estimate.
 *
 * <p>This fills the same {@link MomentumStateUpdater} seam the DRC estimator uses (see
 * {@code DRCKinematicsBasedStateEstimator}: a {@code MomentumStateUpdater} runs once per tick and writes
 * the shared {@link CenterOfMassDataHolder}, which is published to the controller thread; the controller
 * consumes it when {@code hasCenterOfMassPosition()/hasCenterOfMassVelocity()} are set, otherwise it falls
 * back to its own kinematic {@code CenterOfMassJacobian}). Here the "momentum" source is deliberately the
 * simplest one that is consistent with the filter: the whole-body {@link CenterOfMassJacobian} evaluated
 * against the model that the InEKF has just written this tick.</p>
 *
 * <p>Concretely, once {@code InvariantMainStateEstimator} has set the one-DoF joints (q, q̇) and written the
 * root-joint pose <b>and twist</b> from the InEKF, the CoM state is a deterministic function of that state:
 * <pre>
 *   c   = p_B + R_B · &#x1D2E;c(q)                               (FK-exact position)
 *   ċ   = v_B + &#x3C9; &#xD7; (R_B · &#x1D2E;c(q)) + R_B · J_c(q) · q̇      (kinematic velocity)
 * </pre>
 * The {@link CenterOfMassJacobian} bundles both: {@link CenterOfMassJacobian#getCenterOfMass()} returns
 * {@code c} and {@link CenterOfMassJacobian#getCenterOfMassVelocity()} returns {@code ċ}, both expressed in
 * world. Because it reads the root-joint twist, the velocity carries the InEKF base velocity {@code v_B}
 * directly (this is the point of writing it on the estimator model: the value no longer depends on whether
 * the published root twist reaches the controller intact).</p>
 *
 * <p><b>Scope / caveat.</b> The velocity is <em>kinematic only</em> — it is the same quantity the controller
 * would compute in its fallback, just sourced authoritatively from the InEKF. It does <em>not</em> add the
 * ground-reaction / centroidal-momentum fusion that {@code SimpleMomentumStateUpdater} (GRF) or
 * {@code DistributedIMUBasedCenterOfMassStateUpdater} (Alex's default) provide. This class exists so CoM
 * position is InEKF-exact and the balance controller has a single, filter-consistent CoM source; it is also
 * the drop-in seam for a better velocity later (e.g. a GMO/torque-based wrench estimate or a distributed-IMU
 * CoM acceleration), which would replace only the body of {@link #update()} without touching the wiring.</p>
 *
 * <p>Consumes only proprioception already on the model (root state + joints); reads no force/torque sensors,
 * so it runs unchanged on hardware with no foot F/T sensing. Allocation-free after construction.</p>
 */
public class InvariantCenterOfMassUpdater implements MomentumStateUpdater
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   /** Whole-body CoM Jacobian over the root body's subtree, CoM velocity expressed in world. */
   private final CenterOfMassJacobian centerOfMassJacobian;
   /** Shared holder published to the controller; may be null (then this only logs YoVariables). */
   private final CenterOfMassDataHolder centerOfMassDataHolderToUpdate;

   private final YoFramePoint3D yoCenterOfMassPosition = new YoFramePoint3D("invariantCenterOfMassPosition", worldFrame, registry);
   private final YoFrameVector3D yoCenterOfMassVelocity = new YoFrameVector3D("invariantCenterOfMassVelocity", worldFrame, registry);

   /**
    * @param rootJoint                     the floating root joint whose subtree the CoM is computed over.
    *                                      Its pose/twist and the subtree joints must be current (set by the
    *                                      InEKF) before {@link #update()} is called.
    * @param centerOfMassDataHolderToUpdate the shared CoM holder to write each tick; may be {@code null}.
    */
   public InvariantCenterOfMassUpdater(FloatingJointReadOnly rootJoint, CenterOfMassDataHolder centerOfMassDataHolderToUpdate)
   {
      this.centerOfMassDataHolderToUpdate = centerOfMassDataHolderToUpdate;

      RigidBodyReadOnly elevator = rootJoint.getPredecessor();
      centerOfMassJacobian = new CenterOfMassJacobian(elevator, worldFrame);
   }

   @Override
   public void initialize()
   {
      update();
   }

   /** Recomputes CoM position/velocity from the current (InEKF-written) model state and publishes them. */
   @Override
   public void update()
   {
      centerOfMassJacobian.reset();

      yoCenterOfMassPosition.setMatchingFrame(centerOfMassJacobian.getCenterOfMass());
      yoCenterOfMassVelocity.setMatchingFrame(centerOfMassJacobian.getCenterOfMassVelocity());

      if (centerOfMassDataHolderToUpdate != null)
      {
         centerOfMassDataHolderToUpdate.setCenterOfMassPosition(yoCenterOfMassPosition);
         centerOfMassDataHolderToUpdate.setCenterOfMassVelocity(yoCenterOfMassVelocity);
      }
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }
}
