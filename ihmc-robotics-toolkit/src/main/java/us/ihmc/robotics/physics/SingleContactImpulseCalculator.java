package us.ihmc.robotics.physics;

import static us.ihmc.robotics.physics.ContactImpulseTools.computeSlipLambda;
import static us.ihmc.robotics.physics.ContactImpulseTools.isInsideFrictionCone;
import static us.ihmc.robotics.physics.ContactImpulseTools.isInsideFrictionEllipsoid;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.ejml.data.DMatrix3;
import org.ejml.data.DMatrix3x3;
import org.ejml.data.DMatrix4x4;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.fixed.CommonOps_DDF3;
import org.ejml.dense.fixed.CommonOps_DDF4;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.algorithms.MultiBodyResponseCalculator;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialImpulse;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialImpulseBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialImpulseReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;

/**
 * From <i>"Per-Contact Iteration Method for Solving Contact Dynamics"</i>
 * <p>
 * Unconventional matrix ordering: when solving for normal friction moment, the ordering is linear
 * components first and last is the z angular component.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class SingleContactImpulseCalculator implements ImpulseBasedConstraintCalculator
{
   private static final double DEGENERATE_THRESHOLD = 1.0e-6;
   private static final boolean COMPUTE_FRICTION_MOMENT = true;
   private static final int PROBLEM_SIZE = COMPUTE_FRICTION_MOMENT ? 4 : 3;
   private static final double COULOMB_MOMENT_RATIO = 0.3;
   private static final double NEGATIVE_NORMAL_IMPULSE_THRESHOLD = -1.0e-12;

   private double beta1 = 0.35;
   private double beta2 = 0.95;
   private double beta3 = 1.15;
   private double gamma = 1.0e-6;
   private final ContactParameters contactParameters = new ContactParameters(5.0e-5, 0.7, 0.0, 0.0, 0.0, 0.0, 1.0);

   private boolean isFirstUpdate = false;
   private boolean isImpulseZero = false;
   private boolean isContactClosing = false;

   private final ForwardDynamicsCalculator forwardDynamicsCalculatorA;
   private final ForwardDynamicsCalculator forwardDynamicsCalculatorB;

   private final MultiBodyResponseCalculator responseCalculatorA;
   private final MultiBodyResponseCalculator responseCalculatorB;

   private final FramePoint3D pointA = new FramePoint3D();
   private final FramePoint3D pointB = new FramePoint3D();

   private final SpatialVector velocityNoImpulseA = new SpatialVector();
   private final SpatialVector velocityNoImpulseB = new SpatialVector();

   private final SpatialVector velocityDueToOtherImpulseA = new SpatialVector();
   private final SpatialVector velocityDueToOtherImpulseB = new SpatialVector();

   private final SpatialVector velocityA = new SpatialVector();
   private final SpatialVector velocityB = new SpatialVector();
   private final SpatialVector velocityRelative = new SpatialVector();
   private final SpatialVector velocityRelativePrevious = new SpatialVector();
   private final SpatialVector velocityRelativeChange = new SpatialVector();
   /**
    * Velocity used to compute the impulse. It is composed of {@link #velocityRelative} and
    * modifications to inject restitution and error reduction in the solution.
    */
   private final SpatialVector velocitySolverInput = new SpatialVector();

   private final DMatrixRMaj inverseApparentInertiaA = new DMatrixRMaj(PROBLEM_SIZE, PROBLEM_SIZE);
   private final DMatrixRMaj inverseApparentInertiaB = new DMatrixRMaj(PROBLEM_SIZE, PROBLEM_SIZE);

   private final SpatialImpulse impulseA = new SpatialImpulse();
   private final SpatialImpulse impulseB = new SpatialImpulse();
   private final SpatialImpulse impulsePreviousA = new SpatialImpulse();
   private final SpatialImpulse impulseChangeA = new SpatialImpulse();

   private final Point3D contactFramePosition = new Point3D();
   private final Quaternion contactFrameOrientation = new Quaternion();
   private final ReferenceFrame contactFrame;

   private final RigidBodyBasics rootA, rootB;

   private RigidBodyTwistProvider externalRigidBodyTwistModifier;
   private final ImpulseBasedRigidBodyTwistProvider rigidBodyTwistModifierA, rigidBodyTwistModifierB;
   private final ImpulseBasedJointTwistProvider jointTwistModifierA, jointTwistModifierB;

   private CollisionResult collisionResult;
   private RigidBodyBasics contactingBodyA, contactingBodyB;
   private MovingReferenceFrame bodyFrameA, bodyFrameB;

   // Solver data ------------------------------------------------------------------
   /**
    * Contact velocity, extracted from {@link #velocitySolverInput}. When evaluating frictional moment,
    * this is a 4-element vector, the last element being the angular velocity around the z-axis.
    */
   private final DMatrixRMaj c = new DMatrixRMaj(PROBLEM_SIZE, 1);
   /** Linear part of the contact velocity, extracted from {@link #velocitySolverInput} */
   private final DMatrix3 c_linear = new DMatrix3();

   /**
    * The inverse of the apparent inertia matrix.
    * <p>
    * When not solving for frictional moment, this is a 3-by-3 matrix. When solving for the frictional
    * moment, this is a 4-by-4 matrix where the last row and last column are the terms linking the the
    * angular components of the impulse and velocity.
    * </p>
    */
   private final DMatrixRMaj M_inv = new DMatrixRMaj(PROBLEM_SIZE, PROBLEM_SIZE);
   /**
    * The 3-by-3 matrix for the linear part of the inverse of the apparent inertia matrix.
    */
   private final DMatrix3x3 M_linear_inv = new DMatrix3x3();
   /**
    * The 3-by-3 matrix for the linear part of the apparent inertia matrix.
    */
   private final DMatrix3x3 M_linear = new DMatrix3x3();
   /**
    * The 4-by-4 matrix for the linear part of the inverse of the apparent inertia matrix.
    * <p>
    * Only used when solving for the frictional moment.
    * </p>
    * <p>
    * The last row and last column are the terms linking the the angular components of the impulse and
    * velocity.
    * </p>
    */
   private final DMatrix4x4 M_full_inv = new DMatrix4x4();
   /**
    * The 4-by-4 matrix for the linear part of the apparent inertia matrix.
    * <p>
    * Only used when solving for the frictional moment.
    * </p>
    * <p>
    * The last row and last column are the terms linking the the angular components of the impulse and
    * velocity.
    * </p>
    */
   private final DMatrix4x4 M_full = new DMatrix4x4();
   /**
    * The 3-element vector extracted from the 3 first elements of the last row of {@link #M_full}.
    * <p>
    * Only used when solving for the frictional moment.
    * </p>
    * <p>
    * These elements are the coupling part between angular and linear parts.
    * </p>
    */
   private final DMatrix3 M_lin_ang = new DMatrix3();
   /**
    * Solver only used when {@link #M_inv} is not invertible, i.e. degenerate problem.
    */
   private final LinearSolverDense<DMatrixRMaj> svdSolver = LinearSolverFactory_DDRM.pseudoInverse(true);
   /** The linear impulse that fully cancels the contact velocity and may violate the friction law. */
   private final DMatrix3 lambda_linear_v_0 = new DMatrix3();
   /** The linear impulse that best cancels the contact velocity and respect the friction law. */
   private final DMatrix3 lambda_linear = new DMatrix3();
   /**
    * The impulse that fully cancels the contact velocity and may violate the friction law.
    * <p>
    * Only used when for degenerate problems.
    * </p>
    */
   private final DMatrixRMaj lambda_v_0 = new DMatrixRMaj(PROBLEM_SIZE, 1);

   public SingleContactImpulseCalculator(ReferenceFrame rootFrame, RigidBodyBasics rootBodyA, ForwardDynamicsCalculator forwardDynamicsCalculatorA,
                                         RigidBodyBasics rootBodyB, ForwardDynamicsCalculator forwardDynamicsCalculatorB)
   {
      this.forwardDynamicsCalculatorA = forwardDynamicsCalculatorA;
      this.forwardDynamicsCalculatorB = forwardDynamicsCalculatorB;

      rootA = rootBodyA;
      rootB = rootBodyB;

      contactFrame = new ReferenceFrame("contactFrame", rootFrame, true, false)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(contactFrameOrientation, contactFramePosition);
         }
      };

      velocityA.setReferenceFrame(contactFrame);
      velocityB.setReferenceFrame(contactFrame);
      velocityRelative.setReferenceFrame(contactFrame);
      velocityRelativePrevious.setReferenceFrame(contactFrame);
      velocityRelativeChange.setReferenceFrame(contactFrame);

      impulsePreviousA.setReferenceFrame(contactFrame);
      impulseChangeA.setReferenceFrame(contactFrame);

      responseCalculatorA = new MultiBodyResponseCalculator(forwardDynamicsCalculatorA);

      if (forwardDynamicsCalculatorB != null)
         responseCalculatorB = new MultiBodyResponseCalculator(forwardDynamicsCalculatorB);
      else
         responseCalculatorB = null;

      rigidBodyTwistModifierA = new ImpulseBasedRigidBodyTwistProvider(rootFrame, rootA);
      jointTwistModifierA = new ImpulseBasedJointTwistProvider(rootA);

      if (forwardDynamicsCalculatorB != null)
      {
         rigidBodyTwistModifierB = new ImpulseBasedRigidBodyTwistProvider(rootFrame, rootB);
         jointTwistModifierB = new ImpulseBasedJointTwistProvider(rootB);
      }
      else
      {
         rigidBodyTwistModifierB = null;
         jointTwistModifierB = null;
      }
   }

   public void setTolerance(double gamma)
   {
      this.gamma = gamma;
   }

   public void setContactParameters(ContactParametersReadOnly parameters)
   {
      contactParameters.set(parameters);
   }

   @Override
   public void setExternalTwistModifier(RigidBodyTwistProvider externalRigidBodyTwistModifier)
   {
      this.externalRigidBodyTwistModifier = externalRigidBodyTwistModifier;
   }

   public void setCollision(CollisionResult collisionResult)
   {
      Collidable collidableA = collisionResult.getCollidableA();
      Collidable collidableB = collisionResult.getCollidableB();

      if (collidableA.getRootBody() != rootA || collidableB.getRootBody() != rootB)
         throw new IllegalArgumentException("Robot mismatch!");

      this.collisionResult = collisionResult;
      contactingBodyA = collidableA.getRigidBody();
      contactingBodyB = collidableB.getRigidBody();
      bodyFrameA = contactingBodyA.getBodyFixedFrame();
      bodyFrameB = contactingBodyB == null ? null : contactingBodyB.getBodyFixedFrame();
   }

   @Override
   public void initialize(double dt)
   {
      EuclidGeometryTools.orientation3DFromZUpToVector3D(collisionResult.getCollisionAxisForA(), contactFrameOrientation);
      contactFramePosition.set(collisionResult.getPointOnARootFrame());
      contactFrame.update();

      pointA.setIncludingFrame(collisionResult.getCollisionData().getPointOnA());
      pointA.changeFrame(bodyFrameA);

      RigidBodyAccelerationProvider accelerationProviderA = forwardDynamicsCalculatorA.getAccelerationProvider(false);
      predictContactPointSpatialVelocity(dt, rootA, contactingBodyA, accelerationProviderA, pointA, velocityNoImpulseA);
      velocityNoImpulseA.changeFrame(contactFrame);

      pointB.setIncludingFrame(collisionResult.getCollisionData().getPointOnB());

      if (rootB != null)
      {
         pointB.changeFrame(bodyFrameB);
         RigidBodyAccelerationProvider accelerationProviderB = forwardDynamicsCalculatorB.getAccelerationProvider(false);
         predictContactPointSpatialVelocity(dt, rootB, contactingBodyB, accelerationProviderB, pointB, velocityNoImpulseB);
         velocityNoImpulseB.changeFrame(contactFrame);
      }

      velocitySolverInput.setToZero(contactFrame);

      isFirstUpdate = true;
   }

   @Override
   public void updateInertia(List<? extends RigidBodyBasics> rigidBodyTargets, List<? extends JointBasics> jointTargets)
   {
      rigidBodyTwistModifierA.clear(PROBLEM_SIZE);
      jointTwistModifierA.clear(PROBLEM_SIZE);
      if (rigidBodyTargets != null)
         rigidBodyTwistModifierA.addAll(rigidBodyTargets);
      if (jointTargets != null)
         jointTwistModifierA.addAll(jointTargets);

      if (rootB != null)
      {
         rigidBodyTwistModifierB.clear(PROBLEM_SIZE);
         jointTwistModifierB.clear(PROBLEM_SIZE);
         if (rigidBodyTargets != null)
            rigidBodyTwistModifierB.addAll(rigidBodyTargets);
         if (jointTargets != null)
            jointTwistModifierB.addAll(jointTargets);
      }

      // First we evaluate M^-1 that is the inverse of the apparent inertia considering both bodies interacting in this contact.
      computeApparentInertiaInverse(contactingBodyA, responseCalculatorA, rigidBodyTwistModifierA, jointTwistModifierA, inverseApparentInertiaA);

      if (rootB != null)
      {
         computeApparentInertiaInverse(contactingBodyB, responseCalculatorB, rigidBodyTwistModifierB, jointTwistModifierB, inverseApparentInertiaB);
         CommonOps_DDRM.add(inverseApparentInertiaA, inverseApparentInertiaB, M_inv);
      }
      else
      {
         M_inv.set(inverseApparentInertiaA);
      }

      ContactImpulseTools.extract(M_inv, 0, 0, M_linear_inv);
      CommonOps_DDF3.invert(M_linear_inv, M_linear);

      if (COMPUTE_FRICTION_MOMENT)
      {
         M_full_inv.set(M_inv);
         CommonOps_DDF4.invert(M_full_inv, M_full);
      }
   }

   public static void computeContactPointLinearVelocity(double dt, RigidBodyReadOnly rootBody, RigidBodyReadOnly contactingBody,
                                                        RigidBodyAccelerationProvider noVelocityRigidBodyAccelerationProvider,
                                                        FramePoint3DReadOnly contactPoint, FrameVector3DBasics linearVelocityToPack)
   {
      MovingReferenceFrame bodyFixedFrame = contactingBody.getBodyFixedFrame();
      linearVelocityToPack.setReferenceFrame(bodyFixedFrame);

      // Using acceleration relative to the root to avoid integrating the gravitational acceleration.
      SpatialAccelerationReadOnly contactingBodyAcceleration = noVelocityRigidBodyAccelerationProvider.getRelativeAcceleration(rootBody, contactingBody);
      contactingBodyAcceleration.getLinearAccelerationAt(null, contactPoint, linearVelocityToPack);
      linearVelocityToPack.scale(dt);
      double vx = linearVelocityToPack.getX();
      double vy = linearVelocityToPack.getY();
      double vz = linearVelocityToPack.getZ();

      bodyFixedFrame.getTwistOfFrame().getLinearVelocityAt(contactPoint, linearVelocityToPack);
      linearVelocityToPack.add(vx, vy, vz);
   }

   public static void predictContactPointSpatialVelocity(double dt, RigidBodyReadOnly rootBody, RigidBodyReadOnly contactingBody,
                                                         RigidBodyAccelerationProvider noVelocityRigidBodyAccelerationProvider,
                                                         FramePoint3DReadOnly contactPoint, SpatialVectorBasics spatialVelocityToPack)
   {
      MovingReferenceFrame bodyFixedFrame = contactingBody.getBodyFixedFrame();
      spatialVelocityToPack.setReferenceFrame(bodyFixedFrame);

      FixedFrameVector3DBasics angularVelocity = spatialVelocityToPack.getAngularPart();
      FixedFrameVector3DBasics linearVelocity = spatialVelocityToPack.getLinearPart();

      // Using acceleration relative to the root to avoid integrating the gravitational acceleration.
      SpatialAccelerationReadOnly contactingBodyAcceleration = noVelocityRigidBodyAccelerationProvider.getRelativeAcceleration(rootBody, contactingBody);
      angularVelocity.set(contactingBodyAcceleration.getAngularPart());
      contactingBodyAcceleration.getLinearAccelerationAt(null, contactPoint, linearVelocity);
      spatialVelocityToPack.scale(dt);
      double wx = angularVelocity.getX();
      double wy = angularVelocity.getY();
      double wz = angularVelocity.getZ();
      double vx = linearVelocity.getX();
      double vy = linearVelocity.getY();
      double vz = linearVelocity.getZ();

      // Adding the current twist to the integrated acceleration
      packSpatialVelocityAt(bodyFixedFrame.getTwistOfFrame(), contactPoint, spatialVelocityToPack);
      angularVelocity.add(wx, wy, wz);
      linearVelocity.add(vx, vy, vz);
   }

   public static void packSpatialVelocityAt(TwistReadOnly twist, FramePoint3DReadOnly observerPosition, SpatialVectorBasics spatialVelocityToPack)
   {
      spatialVelocityToPack.setReferenceFrame(twist.getReferenceFrame());
      spatialVelocityToPack.getAngularPart().set(twist.getAngularPart());
      twist.getLinearVelocityAt(observerPosition, spatialVelocityToPack.getLinearPart());
   }

   private final FrameVector3D collisionErrorReductionTerm = new FrameVector3D();

   @Override
   public void updateImpulse(double dt, double alpha, boolean ignoreOtherImpulses)
   {
      if (!ignoreOtherImpulses && externalRigidBodyTwistModifier != null)
      {
         // Compute the change in twist due to other impulses.
         packSpatialVelocityAt(externalRigidBodyTwistModifier.getTwistOfBody(contactingBodyA), pointA, velocityDueToOtherImpulseA);
         velocityDueToOtherImpulseA.changeFrame(contactFrame);
         velocityA.add(velocityNoImpulseA, velocityDueToOtherImpulseA);
      }
      else
      {
         velocityA.set(velocityNoImpulseA);
      }

      if (rootB != null)
      {
         if (!ignoreOtherImpulses && externalRigidBodyTwistModifier != null)
         {
            // Compute the change in twist due to other impulses.
            packSpatialVelocityAt(externalRigidBodyTwistModifier.getTwistOfBody(contactingBodyB), pointB, velocityDueToOtherImpulseB);
            velocityDueToOtherImpulseB.changeFrame(contactFrame);
            velocityB.add(velocityNoImpulseB, velocityDueToOtherImpulseB);
         }
         else
         {
            velocityB.set(velocityNoImpulseB);
         }

         velocityRelative.sub(velocityA, velocityB);
      }
      else
      {
         velocityRelative.set(velocityA);
      }

      /*
       * Modifying the contact velocity that the solver is trying to cancel. For instance, the coefficient
       * of restitution is 1.0, the velocity is doubled, which results in an impulse which magnitude is
       * doubled, such that, the post-impact velocity is opposite of the pre-impact velocity along the
       * collision axis.
       */
      velocitySolverInput.set(velocityRelative);
      FixedFrameVector3DBasics linearVelocitySolverInput = velocityRelative.getLinearPart();

      double cr = contactParameters.getCoefficientOfRestitution();
      double rth = contactParameters.getRestitutionThreshold();
      if (cr != 0.0 && velocityRelative.getLinearPart().getZ() <= -rth)
         linearVelocitySolverInput.addZ(cr * (velocityRelative.getLinearPart().getZ() + rth));

      /*
       * Computing the correction term based on the penetration of the two collidables. This assumes that
       * the two shapes are inter-penetrating. The penetration distance is transformed into a velocity
       * that would allow to correct the error in a single tick, this velocity is scaled with the user
       * parameter error-reduction-parameter which is in [0, 1]. The resulting is subtracted to the
       * relative velocity that the solver is trying to cancel, this way the calculator will implicitly
       * account for the error.
       */
      double erp = contactParameters.getErrorReductionParameter();
      if (erp != 0.0)
      {
         collisionErrorReductionTerm.setIncludingFrame(collisionResult.getPointOnBRootFrame());
         collisionErrorReductionTerm.sub(collisionResult.getPointOnARootFrame());
         collisionErrorReductionTerm.changeFrame(contactFrame);
         double normalError = collisionErrorReductionTerm.getZ() - contactParameters.getMinimumPenetration();
         if (normalError > 0.0)
         {
            normalError *= erp / dt;
            linearVelocitySolverInput.subZ(normalError);
         }
      }
      double serp = contactParameters.getSlipErrorReductionParameter();
      if (serp != 0.0)
      {
         collisionErrorReductionTerm.setIncludingFrame(collisionResult.getAccumulatedSlipForA());
         collisionErrorReductionTerm.scale(-serp / dt);
         collisionErrorReductionTerm.changeFrame(contactFrame);
         // Ensure slip error is only tangential to the contact.
         linearVelocitySolverInput.sub(collisionErrorReductionTerm.getX(), collisionErrorReductionTerm.getY(), 0.0);
      }

      isContactClosing = linearVelocitySolverInput.getZ() < 0.0;
      impulseA.setToZero(bodyFrameA, contactFrame);

      if (isContactClosing)
      { // Closing contact, impulse needs to be calculated.
         solveImpulseGeneral(contactParameters.getCoefficientOfFriction(), impulseA);
      }

      if (impulseA.getLinearPart().getZ() < 0.0)
         throw new IllegalStateException("Malformed impulse");

      /*
       * Based on the documentation of ODE, this seems to be the way the constraint force mixing should be
       * applied.
       */
      if (contactParameters.getConstraintForceMixing() != 1.0)
         impulseA.scale(contactParameters.getConstraintForceMixing());

      if (isFirstUpdate)
      {
         if (isContactClosing)
         {
            impulseChangeA.setIncludingFrame(impulseA);
            isImpulseZero = !isContactClosing;
         }
         else
         {
            impulseChangeA.setToZero(bodyFrameA, contactFrame);
            isImpulseZero = true;
         }
      }
      else
      {
         impulseA.interpolate(impulsePreviousA, impulseA, alpha);
         impulseChangeA.sub(impulseA, impulsePreviousA);
         isImpulseZero = impulseA.getLinearPart().length() < 1.0e-6;
      }

      if (isImpulseZero)
      {
         if (rootB != null)
         {
            impulseB.setToZero(contactingBodyB.getBodyFixedFrame(), impulseA.getReferenceFrame());
         }
      }
      else
      {
         if (rootB != null)
         {
            impulseB.setIncludingFrame(contactingBodyB.getBodyFixedFrame(), impulseA);
            impulseB.negate();
         }
      }

      impulsePreviousA.setIncludingFrame(impulseA);

      if (isFirstUpdate)
      {
         velocityRelativePrevious.set(velocityRelative);
         velocityRelativeChange.set(velocityRelative);
      }
      else
      {
         velocityRelativeChange.sub(velocityRelative, velocityRelativePrevious);
         velocityRelativePrevious.set(velocityRelative);
      }

      isFirstUpdate = false;
   }

   public void solveImpulseGeneral(double mu, FixedFrameSpatialImpulseBasics impulseToPack)
   {
      if (EuclidCoreTools.isZero(mu, 1.0e-12))
      { // Trivial case, i.e. there is no friction => the impulse is along the collision axis.
         impulseToPack.getLinearPart().setZ(-velocitySolverInput.getLinearPart().getZ() / M_inv.get(2, 2));
         return;
      }

      if (CommonOps_DDRM.det(M_inv) <= DEGENERATE_THRESHOLD)
      {
         solveImpulseDegenerate(mu, impulseToPack);
         return;
      }

      velocitySolverInput.getLinearPart().get(c_linear);
      boolean isSlipping = solveLinearImpulse(mu, impulseToPack);

      /*
       * When the contact is slipping and that we're considering only the linear part of it, we skip the
       * evaluation of the frictional moment and will leave the angular impulse to zero.
       */
      if (!isSlipping)
      {
         if (COMPUTE_FRICTION_MOMENT)
         {
            solveAngularImpulse(mu, impulseToPack);
         }
      }
   }

   public boolean solveLinearImpulse(double mu, FixedFrameSpatialImpulseBasics impulseToPack)
   {
      CommonOps_DDF3.mult(M_linear, c_linear, lambda_linear_v_0);
      CommonOps_DDF3.changeSign(lambda_linear_v_0);

      if (lambda_linear_v_0.a3 > NEGATIVE_NORMAL_IMPULSE_THRESHOLD && isInsideFrictionCone(mu, lambda_linear_v_0))
      { // Contact is sticking, i.e. satisfies Coulomb's friction cone while canceling velocity.
         impulseToPack.getLinearPart().set(lambda_linear_v_0);
         return false;
      }
      else
      { // Contact is slipping, that's the though case.
         computeSlipLambda(beta1, beta2, beta3, gamma, mu, M_linear_inv, lambda_linear_v_0, c_linear, lambda_linear, false);
         impulseToPack.getLinearPart().set(lambda_linear);
         return true;
      }
   }

   /**
    * This method attempts to compute the linear and z-angular impulse that cancels the contact
    * velocity (linear and z-angular).
    * <p>
    * In the case that canceling the z-angular velocity cannot be achieved, i.e. resulting impulse
    * violates the generalized friction law, the linear part of the impulse is prioritized over the
    * angular. The current implementation performs a bisection on the z-angular velocity to find the
    * maximum velocity that can be cancelled while satisfying the friction law.
    * </p>
    * <p>
    * This method will not compute an optimal solution in the sense the that the solution is not
    * guaranteed to provide maximum dissipation.
    * </p>
    */
   private void solveAngularImpulse(double mu, FixedFrameSpatialImpulseBasics impulseToPack)
   {
      /*
       * @formatter:off
       * / lambda_linear  \ = - / M_linear    M_lin_ang \ / c_linear  \
       * \ lambda_angular /     \ M_lin_ang^T M_angular / \ c_angular /
       * 
       * We first compute the part of the system that is independent from c_angular:
       * lambda_linear_decoupled = -M_linear * c_linear
       * lambda_angular_coupling = -M_lin_ang . c_linear
       * When 
       * @formatter:on
       */
      DMatrix3 lambda_linear_decoupled = lambda_linear_v_0;
      lambda_linear_decoupled.a1 = -M_full.a11 * c_linear.a1 - M_full.a12 * c_linear.a2 - M_full.a13 * c_linear.a3;
      lambda_linear_decoupled.a2 = -M_full.a21 * c_linear.a1 - M_full.a22 * c_linear.a2 - M_full.a23 * c_linear.a3;
      lambda_linear_decoupled.a3 = -M_full.a31 * c_linear.a1 - M_full.a32 * c_linear.a2 - M_full.a33 * c_linear.a3;

      M_lin_ang.set(M_full.a14, M_full.a24, M_full.a34);
      double lambda_angular_coupling = -CommonOps_DDF3.dot(M_lin_ang, c_linear);

      // We first check that the contact is sticking when ignoring the angular velocity, if not we abort.
      if (lambda_linear_decoupled.a3 < NEGATIVE_NORMAL_IMPULSE_THRESHOLD
            || !isInsideFrictionEllipsoid(mu, lambda_linear_decoupled, lambda_angular_coupling, COULOMB_MOMENT_RATIO))
         return; // Unable to solve this for now, falling back to solution without friction moment.

      double M_angular = M_full.get(3, 3);
      double c_angular = velocitySolverInput.getAngularPartZ();

      ContactImpulseTools.scaleAdd(-c_angular, M_lin_ang, lambda_linear_decoupled, lambda_linear);
      double lambda_angular = lambda_angular_coupling - M_angular * c_angular;

      if (lambda_linear.a3 > NEGATIVE_NORMAL_IMPULSE_THRESHOLD && isInsideFrictionEllipsoid(mu, lambda_linear, lambda_angular, COULOMB_MOMENT_RATIO))
      {
         // The contact is sticking, we're done.
         impulseToPack.getLinearPart().set(lambda_linear);
         impulseToPack.getAngularPart().setZ(lambda_angular);
         return;
      }

      /*
       * The contact sticking is slipping when countering 100% of c_angular. We start a bisection to find
       * the max value for c_angular for which the impulse remains within the friction ellipsoid.
       */
      // The lower bound is always sticking
      double c_angular_lo = 0.0;
      // The upper bound is always slipping
      double c_angular_hi = c_angular;
      double lambda_lo_x = lambda_linear_decoupled.a1;
      double lambda_lo_y = lambda_linear_decoupled.a2;
      double lambda_lo_z = lambda_linear_decoupled.a3;
      double lambda_lo_zz = lambda_angular_coupling;
      int iteration = 0;

      while (true)
      {
         iteration++;

         if (Math.abs(c_angular_hi - c_angular_lo) < gamma)
         {
            impulseToPack.getLinearPart().set(lambda_lo_x, lambda_lo_y, lambda_lo_z);
            impulseToPack.getAngularPart().setZ(lambda_lo_zz);
            return;
         }

         if (iteration > 1000)
         {
            throw new IllegalStateException("Failed to computed friction moment");
         }

         double c_angular_mid = 0.5 * (c_angular_lo + c_angular_hi);

         ContactImpulseTools.scaleAdd(-c_angular_mid, M_lin_ang, lambda_linear_decoupled, lambda_linear);

         if (lambda_linear.a3 < NEGATIVE_NORMAL_IMPULSE_THRESHOLD)
         { // We're slipping
            c_angular_hi = c_angular_mid;
            continue;
         }

         lambda_angular = lambda_angular_coupling - M_angular * c_angular_mid;

         if (!isInsideFrictionEllipsoid(mu, lambda_linear, lambda_angular, COULOMB_MOMENT_RATIO))
         { // We're slipping
            c_angular_hi = c_angular_mid;
            continue;
         }

         // We're sticking
         c_angular_lo = c_angular_mid;
         lambda_lo_x = lambda_linear.a1;
         lambda_lo_y = lambda_linear.a2;
         lambda_lo_z = lambda_linear.a3;
         lambda_lo_zz = lambda_angular;
      }
   }

   /*
    * TODO This is not enough to cover the degenerate case, need to decompose the problem to work in
    * reduced space, i.e. either 2D or 1D.
    */
   public void solveImpulseDegenerate(double mu, FixedFrameSpatialImpulseBasics impulseToPack)
   {
      lambda_v_0.reshape(PROBLEM_SIZE, 1);
      c.reshape(PROBLEM_SIZE, 1);
      velocitySolverInput.getLinearPart().get(c);

      if (COMPUTE_FRICTION_MOMENT)
         c.set(3, velocitySolverInput.getAngularPart().getZ());
      svdSolver.setA(M_inv);
      svdSolver.solve(c, lambda_v_0);
      CommonOps_DDRM.changeSign(lambda_v_0);

      if (lambda_v_0.get(2) < NEGATIVE_NORMAL_IMPULSE_THRESHOLD)
         throw new IllegalStateException("Unable to fully solve degenerate case. Need to be improved.");

      if (COMPUTE_FRICTION_MOMENT)
      {
         if (!isInsideFrictionEllipsoid(mu, lambda_v_0, COULOMB_MOMENT_RATIO))
            throw new IllegalStateException("Unable to fully solve degenerate case. Need to be improved.");
      }
      else
      {
         if (!isInsideFrictionCone(mu, lambda_v_0))
            throw new IllegalStateException("Unable to fully solve degenerate case. Need to be improved.");
      }
   }

   @Override
   public void updateTwistModifiers()
   {
      if (isImpulseZero)
      {
         rigidBodyTwistModifierA.setImpulseToZero();
         jointTwistModifierA.setImpulseToZero();

         if (rootB != null)
         {
            rigidBodyTwistModifierB.setImpulseToZero();
            jointTwistModifierB.setImpulseToZero();
         }
      }
      else
      {
         if (COMPUTE_FRICTION_MOMENT)
         {
            rigidBodyTwistModifierA.setImpulse(impulseA.getLinearPart(), impulseA.getAngularPartZ());
            jointTwistModifierA.setImpulse(impulseA.getLinearPart(), impulseA.getAngularPartZ());

            if (rootB != null)
            {
               rigidBodyTwistModifierB.setImpulse(impulseB.getLinearPart(), impulseB.getAngularPartZ());
               jointTwistModifierB.setImpulse(impulseB.getLinearPart(), impulseB.getAngularPartZ());
            }
         }
         else
         {
            rigidBodyTwistModifierA.setImpulse(impulseA.getLinearPart());
            jointTwistModifierA.setImpulse(impulseA.getLinearPart());

            if (rootB != null)
            {
               rigidBodyTwistModifierB.setImpulse(impulseB.getLinearPart());
               jointTwistModifierB.setImpulse(impulseB.getLinearPart());
            }
         }
      }
   }

   @Override
   public void finalizeImpulse()
   {
      responseCalculatorA.applyRigidBodyImpulse(contactingBodyA, impulseA);
      if (rootB != null)
         responseCalculatorB.applyRigidBodyImpulse(contactingBodyB, impulseB);
   }

   private final SpatialImpulse testImpulse = new SpatialImpulse();
   private final Twist testTwist = new Twist();

   private void computeApparentInertiaInverse(RigidBodyBasics body, MultiBodyResponseCalculator calculator,
                                              ImpulseBasedRigidBodyTwistProvider rigidBodyTwistModifierToUpdate,
                                              ImpulseBasedJointTwistProvider jointTwistModifierToUpdate, DMatrixRMaj inertiaMatrixToPack)
   {
      calculator.reset();
      inertiaMatrixToPack.reshape(PROBLEM_SIZE, PROBLEM_SIZE);
      RigidBodyTwistProvider twistChangeProvider = calculator.getTwistChangeProvider();

      // Computing the x,y,z linear components
      for (int axis = 0; axis < 3; axis++)
      {
         testImpulse.setIncludingFrame(body.getBodyFixedFrame(), contactFrame, EuclidCoreTools.zeroVector3D, Axis3D.values[axis]);

         if (!calculator.applyRigidBodyImpulse(body, testImpulse))
            throw new IllegalStateException("Something went wrong with the response calculator");

         testTwist.setIncludingFrame(twistChangeProvider.getTwistOfBody(body));
         testTwist.changeFrame(contactFrame);
         testTwist.getLinearPart().get(0, axis, inertiaMatrixToPack);
         if (COMPUTE_FRICTION_MOMENT)
            inertiaMatrixToPack.set(3, axis, testTwist.getAngularPartZ());

         for (RigidBodyBasics externalTarget : rigidBodyTwistModifierToUpdate.getRigidBodies())
         {
            DMatrixRMaj externalInertiaMatrix = rigidBodyTwistModifierToUpdate.getApparentInertiaMatrixInverse(externalTarget);
            twistChangeProvider.getTwistOfBody(externalTarget).get(0, axis, externalInertiaMatrix);
         }

         for (JointBasics externalTarget : jointTwistModifierToUpdate.getJoints())
         {
            DMatrixRMaj externalInertiaMatrix = jointTwistModifierToUpdate.getApparentInertiaMatrixInverse(externalTarget);
            CommonOps_DDRM.insert(calculator.getJointTwistChange(externalTarget), externalInertiaMatrix, 0, axis);
         }

         calculator.reset();
      }

      if (COMPUTE_FRICTION_MOMENT)
      {// Computing the z angular component
         testImpulse.setIncludingFrame(body.getBodyFixedFrame(), contactFrame, Axis3D.Z, EuclidCoreTools.zeroVector3D);

         if (!calculator.applyRigidBodyImpulse(body, testImpulse))
            throw new IllegalStateException("Something went wrong with the response calculator");

         testTwist.setIncludingFrame(twistChangeProvider.getTwistOfBody(body));
         testTwist.changeFrame(contactFrame);
         testTwist.getLinearPart().get(0, 3, inertiaMatrixToPack);
         inertiaMatrixToPack.set(3, 3, testTwist.getAngularPartZ());

         for (RigidBodyBasics externalTarget : rigidBodyTwistModifierToUpdate.getRigidBodies())
         {
            DMatrixRMaj externalInertiaMatrix = rigidBodyTwistModifierToUpdate.getApparentInertiaMatrixInverse(externalTarget);
            twistChangeProvider.getTwistOfBody(externalTarget).get(0, 3, externalInertiaMatrix);
         }

         for (JointBasics externalTarget : jointTwistModifierToUpdate.getJoints())
         {
            DMatrixRMaj externalInertiaMatrix = jointTwistModifierToUpdate.getApparentInertiaMatrixInverse(externalTarget);
            CommonOps_DDRM.insert(calculator.getJointTwistChange(externalTarget), externalInertiaMatrix, 0, 3);
         }

         calculator.reset();
      }
   }

   public void readExternalWrench(double dt, List<ExternalWrenchReader> externalWrenchReaders)
   {
      externalWrenchReaders.forEach(reader -> readExternalWrench(dt, reader));
   }

   public void readExternalWrench(double dt, ExternalWrenchReader externalWrenchReader)
   {
      Wrench externalWrenchA = new Wrench();
      externalWrenchA.setIncludingFrame(impulseA.getBodyFrame(), impulseA);
      externalWrenchA.scale(1.0 / dt);
      externalWrenchA.changeFrame(bodyFrameA);
      externalWrenchReader.readExternalWrench(contactingBodyA, externalWrenchA);

      if (rootB != null)
      {
         Wrench externalWrenchB = new Wrench();
         externalWrenchB.setIncludingFrame(impulseB.getBodyFrame(), impulseB);
         externalWrenchB.scale(1.0 / dt);
         externalWrenchB.changeFrame(bodyFrameB);
         externalWrenchReader.readExternalWrench(contactingBodyB, externalWrenchB);
      }
   }

   @Override
   public double getImpulseUpdate()
   {
      return impulseChangeA.length();
   }

   @Override
   public double getVelocityUpdate()
   {
      return velocityRelativeChange.length();
   }

   @Override
   public boolean isConstraintActive()
   {
      return !isImpulseZero;
   }

   public boolean isContactClosing()
   {
      return isContactClosing;
   }

   public CollisionResult getCollisionResult()
   {
      return collisionResult;
   }

   public RigidBodyBasics getContactingBodyA()
   {
      return contactingBodyA;
   }

   public ForwardDynamicsCalculator getForwardDynamicsCalculatorA()
   {
      return forwardDynamicsCalculatorA;
   }

   public RigidBodyBasics getContactingBodyB()
   {
      return contactingBodyB;
   }

   public ForwardDynamicsCalculator getForwardDynamicsCalculatorB()
   {
      return forwardDynamicsCalculatorB;
   }

   @Override
   public int getNumberOfRobotsInvolved()
   {
      return responseCalculatorB == null ? 1 : 2;
   }

   @Override
   public RigidBodyBasics getRootBody(int index)
   {
      return index == 0 ? rootA : rootB;
   }

   @Override
   public RigidBodyTwistProvider getRigidBodyTwistChangeProvider(int index)
   {
      return index == 0 ? rigidBodyTwistModifierA : rigidBodyTwistModifierB;
   }

   @Override
   public JointStateProvider getJointTwistChangeProvider(int index)
   {
      return index == 0 ? jointTwistModifierA : jointTwistModifierB;
   }

   @Override
   public DMatrixRMaj getJointVelocityChange(int index)
   {
      return index == 0 ? getJointVelocityChangeA() : getJointVelocityChangeB();
   }

   public DMatrixRMaj getJointVelocityChangeA()
   {
      if (!isConstraintActive())
         return null;

      return responseCalculatorA.propagateImpulse();
   }

   public DMatrixRMaj getJointVelocityChangeB()
   {
      if (rootB == null || !isConstraintActive())
         return null;

      return responseCalculatorB.propagateImpulse();
   }

   public SpatialImpulseReadOnly getImpulseA()
   {
      return impulseA;
   }

   public SpatialImpulseReadOnly getImpulseB()
   {
      return impulseB;
   }

   public FramePoint3DReadOnly getPointA()
   {
      return pointA;
   }

   public FramePoint3DReadOnly getPointB()
   {
      return pointB;
   }

   public SpatialVectorReadOnly getVelocityRelative()
   {
      return velocityRelative;
   }

   public SpatialVectorReadOnly getVelocitySolverInput()
   {
      return velocitySolverInput;
   }

   public DMatrixRMaj getCollisionMatrix()
   {
      return M_inv;
   }

   public SpatialVectorReadOnly getVelocityNoImpulseA()
   {
      return velocityNoImpulseA;
   }

   public SpatialVectorReadOnly getVelocityNoImpulseB()
   {
      return velocityNoImpulseB;
   }

   public SpatialVectorReadOnly getVelocityDueToOtherImpulseA()
   {
      return velocityDueToOtherImpulseA;
   }

   public SpatialVectorReadOnly getVelocityDueToOtherImpulseB()
   {
      return velocityDueToOtherImpulseB;
   }

   public MultiBodyResponseCalculator getResponseCalculatorA()
   {
      return responseCalculatorA;
   }

   public MultiBodyResponseCalculator getResponseCalculatorB()
   {
      return responseCalculatorB;
   }

   public ContactParametersBasics getContactParameters()
   {
      return contactParameters;
   }

   @Override
   public List<? extends JointBasics> getJointTargets()
   {
      return Collections.emptyList();
   }

   @Override
   public List<? extends RigidBodyBasics> getRigidBodyTargets()
   {
      if (rootB == null)
         return Collections.singletonList(contactingBodyA);
      else
         return Arrays.asList(contactingBodyA, contactingBodyB);
   }

   public void printForUnitTest()
   {
      System.err.println(ContactImpulseTools.toStringForUnitTest(beta1,
                                                                 beta2,
                                                                 beta3,
                                                                 gamma,
                                                                 contactParameters.getCoefficientOfFriction(),
                                                                 M_inv,
                                                                 lambda_v_0,
                                                                 c));
   }

   @Override
   public String toString()
   {
      return "Collidables [A: " + PhysicsEngineTools.collidableSimpleName(collisionResult.getCollidableA()) + ", B: "
            + PhysicsEngineTools.collidableSimpleName(collisionResult.getCollidableB()) + "], velocity relative: " + velocityRelative + ", impulse A: "
            + impulseA.getLinearPart();
   }
}
