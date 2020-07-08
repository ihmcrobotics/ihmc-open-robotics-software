package us.ihmc.robotics.physics;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

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
   private final ContactParameters contactParameters = new ContactParameters();

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

   private final DMatrixRMaj inverseApparentInertiaA = new DMatrixRMaj(4, 4);
   private final DMatrixRMaj inverseApparentInertiaB = new DMatrixRMaj(4, 4);

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

   private final SingleContactImpulseSolver solver = new SingleContactImpulseSolver();

   /**
    * The inverse of the apparent inertia matrix.
    * <p>
    * When not solving for frictional moment, this is a 3-by-3 matrix. When solving for the frictional
    * moment, this is a 4-by-4 matrix where the last row and last column are the terms linking the the
    * angular components of the impulse and velocity.
    * </p>
    */
   private final DMatrixRMaj M_inv = new DMatrixRMaj(4, 4);

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

      setContactParameters(ContactParameters.defaultIneslasticContactParameters(true));
   }

   public void setTolerance(double gamma)
   {
      solver.setTolerance(gamma);
   }

   public void setContactParameters(ContactParametersReadOnly parameters)
   {
      contactParameters.set(parameters);
      solver.setEnableFrictionMoment(parameters.getComputeFrictionMoment());
      solver.setCoulombMomentRatio(parameters.getCoulombMomentFrictionRatio());
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
      rigidBodyTwistModifierA.clear(solver.getProblemSize());
      jointTwistModifierA.clear(solver.getProblemSize());
      if (rigidBodyTargets != null)
         rigidBodyTwistModifierA.addAll(rigidBodyTargets);
      if (jointTargets != null)
         jointTwistModifierA.addAll(jointTargets);

      if (rootB != null)
      {
         rigidBodyTwistModifierB.clear(solver.getProblemSize());
         jointTwistModifierB.clear(solver.getProblemSize());
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

      solver.reset();
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
         velocityA.set(velocityNoImpulseA);
         velocityA.add(velocityDueToOtherImpulseA);
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
            velocityB.set(velocityNoImpulseB);
            velocityB.add(velocityDueToOtherImpulseB);
         }
         else
         {
            velocityB.set(velocityNoImpulseB);
         }

         velocityRelative.set(velocityA);
         velocityRelative.sub(velocityB);
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
      FixedFrameVector3DBasics linearVelocitySolverInput = velocitySolverInput.getLinearPart();

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

      isContactClosing = linearVelocitySolverInput.getZ() < 0.0;
      impulseA.setToZero(bodyFrameA, contactFrame);

      if (isContactClosing)
      { // Closing contact, impulse needs to be calculated.
         solver.solveImpulseGeneral(velocitySolverInput, M_inv, contactParameters.getCoefficientOfFriction(), impulseA);
      }

      if (impulseA.getLinearPart().getZ() < 0.0)
         throw new IllegalStateException("Malformed impulse");

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
         impulseA.getLinearPart().interpolate(impulsePreviousA.getLinearPart(), impulseA.getLinearPart(), alpha);
         if (contactParameters.getComputeFrictionMoment())
            impulseA.getAngularPart().setZ(EuclidCoreTools.interpolate(impulsePreviousA.getAngularPartZ(), impulseA.getAngularPartZ(), alpha));
         impulseChangeA.set(impulseA);
         impulseChangeA.sub(impulsePreviousA);
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
         velocityRelativeChange.set(velocityRelative);
         velocityRelativeChange.sub(velocityRelativePrevious);
         velocityRelativePrevious.set(velocityRelative);
      }

      isFirstUpdate = false;
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
         if (contactParameters.getComputeFrictionMoment())
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
      inertiaMatrixToPack.reshape(solver.getProblemSize(), solver.getProblemSize());
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
         if (contactParameters.getComputeFrictionMoment())
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

      if (contactParameters.getComputeFrictionMoment())
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
      return EuclidCoreTools.norm(impulseChangeA.getLinearPartX(),
                                  impulseChangeA.getLinearPartY(),
                                  impulseChangeA.getLinearPartZ(),
                                  impulseChangeA.getAngularPartZ());
   }

   @Override
   public double getVelocityUpdate()
   {
      return EuclidCoreTools.norm(velocityRelativeChange.getLinearPartX(),
                                  velocityRelativeChange.getLinearPartY(),
                                  velocityRelativeChange.getLinearPartZ(),
                                  velocityRelativeChange.getAngularPartZ());
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
      solver.printForUnitTest(M_inv, contactParameters.getCoefficientOfFriction());
   }

   @Override
   public String toString()
   {
      return "Collidables [A: " + PhysicsEngineTools.collidableSimpleName(collisionResult.getCollidableA()) + ", B: "
            + PhysicsEngineTools.collidableSimpleName(collisionResult.getCollidableB()) + "], velocity relative: " + velocityRelative + ", impulse A: "
            + impulseA.getLinearPart();
   }
}
