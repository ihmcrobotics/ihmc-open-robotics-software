package us.ihmc.robotics.physics;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.algorithms.MultiBodyResponseCalculator;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialImpulse;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialImpulseReadOnly;

/**
 * From <i>"Per-Contact Iteration Method for Solving Contact Dynamics"</i>
 *
 * @author Sylvain Bertrand
 */
public class SingleContactImpulseCalculator
{
   private double coefficientOfFriction = 0.7;
   private double beta1 = 0.35;
   private double beta2 = 0.95;
   private double beta3 = 1.15;
   private double gamma = 1.0e-6;
   private double springConstant = 5.0;

   private boolean isInitialized = false;
   private boolean isInertiaUpToDate = false;

   private final ForwardDynamicsCalculator forwardDynamicsCalculatorA;
   private final ForwardDynamicsCalculator forwardDynamicsCalculatorB;

   private final MultiBodyResponseCalculator collisionCalculatorA;
   private final MultiBodyResponseCalculator collisionCalculatorB;

   private final FramePoint3D contactPointA = new FramePoint3D();
   private final FramePoint3D contactPointB = new FramePoint3D();

   private final FrameVector3D noImpulseVelocityA = new FrameVector3D();
   private final FrameVector3D noImpulseVelocityB = new FrameVector3D();

   private final FrameVector3D velocityDueToOtherImpulseA = new FrameVector3D();
   private final FrameVector3D velocityDueToOtherImpulseB = new FrameVector3D();

   private final FrameVector3D contactPointVelocityA = new FrameVector3D();
   private final FrameVector3D contactPointVelocityB = new FrameVector3D();
   private final FrameVector3D contactVelocity = new FrameVector3D();
   private final FrameVector3D previousContactVelocity = new FrameVector3D();
   private final FrameVector3D contactVelocityChange = new FrameVector3D();

   private final DenseMatrix64F inverseApparentInertiaA = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F inverseApparentInertiaB = new DenseMatrix64F(3, 3);

   private final SpatialImpulse impulseA = new SpatialImpulse();
   private final SpatialImpulse impulseB = new SpatialImpulse();
   private final FrameVector3D previousImpulseA = new FrameVector3D();
   private final FrameVector3D impulseChangeA = new FrameVector3D();

   private final Point3D contactFramePosition = new Point3D();
   private final Quaternion contactFrameOrientation = new Quaternion();
   private final ReferenceFrame contactFrame;

   private final DenseMatrix64F c = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F M_inv = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F lambda_v_0 = new DenseMatrix64F(3, 1);
   private final LinearSolver<DenseMatrix64F> linearSolver = LinearSolverFactory.symmPosDef(3);
   private final LinearSolver<DenseMatrix64F> svdSolver = LinearSolverFactory.pseudoInverse(true);
   private final DenseMatrix64F M_inv_solverCopy = new DenseMatrix64F(3, 3);

   private final RigidBodyBasics rootA, rootB;
   private final RigidBodyBasics contactingBodyA, contactingBodyB;
   private final MovingReferenceFrame bodyFrameA, bodyFrameB;

   private final DenseMatrix64F jointVelocityChangeA = new DenseMatrix64F(20, 1);
   private final DenseMatrix64F jointVelocityChangeB = new DenseMatrix64F(20, 1);

   private double dt;

   private CollisionResult collisionResult;
   private RigidBodyTwistProvider externalRigidBodyTwistModifierA, externalRigidBodyTwistModifierB;

   private boolean isContactClosing;

   public SingleContactImpulseCalculator(CollisionResult collisionResult, ReferenceFrame rootFrame, double dt,
                                         ForwardDynamicsCalculator forwardDynamicsCalculatorA, ForwardDynamicsCalculator forwardDynamicsCalculatorB)
   {
      this.collisionResult = collisionResult;
      this.dt = dt;
      this.forwardDynamicsCalculatorA = forwardDynamicsCalculatorA;
      this.forwardDynamicsCalculatorB = forwardDynamicsCalculatorB;

      rootA = collisionResult.getCollidableA().getRootBody();
      rootB = collisionResult.getCollidableB().getRootBody();
      contactingBodyA = collisionResult.getCollidableA().getRigidBody();
      contactingBodyB = collisionResult.getCollidableB().getRigidBody();
      bodyFrameA = contactingBodyA.getBodyFixedFrame();
      bodyFrameB = contactingBodyB == null ? null : contactingBodyB.getBodyFixedFrame();

      contactFrame = new ReferenceFrame("contactFrame", rootFrame, true, false)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(contactFrameOrientation, contactFramePosition);
         }
      };

      contactPointVelocityA.setReferenceFrame(contactFrame);
      contactPointVelocityB.setReferenceFrame(contactFrame);
      contactVelocity.setReferenceFrame(contactFrame);
      previousContactVelocity.setReferenceFrame(contactFrame);
      contactVelocityChange.setReferenceFrame(contactFrame);

      previousImpulseA.setReferenceFrame(contactFrame);
      impulseChangeA.setReferenceFrame(contactFrame);

      collisionCalculatorA = new MultiBodyResponseCalculator(forwardDynamicsCalculatorA);

      if (forwardDynamicsCalculatorB != null)
         collisionCalculatorB = new MultiBodyResponseCalculator(forwardDynamicsCalculatorB);
      else
         collisionCalculatorB = null;
   }

   public void setSpringConstant(double springConstant)
   {
      this.springConstant = springConstant;
   }

   public void setTolerance(double gamma)
   {
      this.gamma = gamma;
   }

   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction = coefficientOfFriction;
   }

   public void setExternalTwistModifier(RigidBodyTwistProvider externalRigidBodyTwistModifier)
   {
      setExternalTwistModifiers(externalRigidBodyTwistModifier, externalRigidBodyTwistModifier);
   }

   public void setExternalTwistModifiers(RigidBodyTwistProvider externalRigidBodyTwistModifierA, RigidBodyTwistProvider externalRigidBodyTwistModifierB)
   {
      this.externalRigidBodyTwistModifierA = externalRigidBodyTwistModifierA;
      this.externalRigidBodyTwistModifierB = externalRigidBodyTwistModifierB;
   }

   public void initialize()
   {
      if (isInitialized)
         return;

      EuclidGeometryTools.orientation3DFromZUpToVector3D(collisionResult.getCollisionAxisForA(), contactFrameOrientation);
      contactFramePosition.set(collisionResult.getPointOnARootFrame());
      contactFrame.update();

      contactPointA.setIncludingFrame(collisionResult.getCollisionData().getPointOnA());
      contactPointA.changeFrame(bodyFrameA);

      computeContactPointVelocity(dt, rootA, contactingBodyA, forwardDynamicsCalculatorA.getAccelerationProvider(), contactPointA, noImpulseVelocityA);
      noImpulseVelocityA.changeFrame(contactFrame);

      if (rootB != null)
      {
         contactPointB.setIncludingFrame(collisionResult.getCollisionData().getPointOnB());
         contactPointB.changeFrame(bodyFrameB);
         computeContactPointVelocity(dt, rootB, contactingBodyB, forwardDynamicsCalculatorB.getAccelerationProvider(), contactPointB, noImpulseVelocityB);
         noImpulseVelocityB.changeFrame(contactFrame);
      }

      isInertiaUpToDate = false;
      isInitialized = true;
   }

   static void computeContactPointVelocity(double dt, RigidBodyReadOnly rootBody, RigidBodyReadOnly contactingBody,
                                           RigidBodyAccelerationProvider accelerationProvider, FramePoint3DReadOnly contactPoint,
                                           FrameVector3DBasics linearVelocityToPack)
   {
      MovingReferenceFrame bodyFixedFrame = contactingBody.getBodyFixedFrame();

      SpatialAccelerationReadOnly contactingBodyAcceleration = accelerationProvider.getRelativeAcceleration(rootBody, contactingBody);
      contactingBodyAcceleration.getLinearAccelerationAt(null, contactPoint, linearVelocityToPack);
      linearVelocityToPack.scale(dt);
      double vx = linearVelocityToPack.getX();
      double vy = linearVelocityToPack.getY();
      double vz = linearVelocityToPack.getZ();

      bodyFixedFrame.getTwistOfFrame().getLinearVelocityAt(contactPoint, linearVelocityToPack);
      linearVelocityToPack.add(vx, vy, vz);
   }

   public void computeImpulse()
   {
      isInitialized = false;
      updateImpulse(1.0);
   }

   private final FrameVector3D collisionPositionTerm = new FrameVector3D();

   public void updateImpulse(double alpha)
   {
      boolean isFirstUpdate = !isInitialized;
      if (!isInitialized)
         initialize();

      if (externalRigidBodyTwistModifierA != null)
      {
         velocityDueToOtherImpulseA.setIncludingFrame(externalRigidBodyTwistModifierA.getLinearVelocityOfBodyFixedPoint(contactingBodyA, contactPointA));
         velocityDueToOtherImpulseA.changeFrame(contactFrame);
         contactPointVelocityA.add(noImpulseVelocityA, velocityDueToOtherImpulseA);
      }
      else
      {
         contactPointVelocityA.set(noImpulseVelocityA);
      }

      if (rootB != null)
      {
         if (externalRigidBodyTwistModifierB != null)
         {
            velocityDueToOtherImpulseB.setIncludingFrame(externalRigidBodyTwistModifierA.getLinearVelocityOfBodyFixedPoint(contactingBodyB, contactPointB));
            velocityDueToOtherImpulseB.changeFrame(contactFrame);
            contactPointVelocityB.add(noImpulseVelocityB, velocityDueToOtherImpulseB);
         }
         else
         {
            contactPointVelocityB.set(noImpulseVelocityB);
         }

         contactVelocity.sub(contactPointVelocityA, contactPointVelocityB);
      }
      else
      {
         contactVelocity.set(contactPointVelocityA);
      }

      if (isFirstUpdate)
      {
         previousContactVelocity.set(contactVelocity);
         contactVelocityChange.set(contactVelocity);
      }
      else
      {
         contactVelocityChange.sub(contactVelocity, previousContactVelocity);
         previousContactVelocity.set(contactVelocity);
      }

      impulseA.setToZero(bodyFrameA, contactFrame);

      isContactClosing = contactVelocity.getZ() < 0.0;

      if (isContactClosing)
      { // Closing contact, impulse needs to be calculated.
         collisionPositionTerm.setIncludingFrame(collisionResult.getPointOnBRootFrame());
         collisionPositionTerm.sub(collisionResult.getPointOnARootFrame());
         collisionPositionTerm.scale(springConstant);
         collisionPositionTerm.changeFrame(contactVelocity.getReferenceFrame());
         contactVelocity.sub(collisionPositionTerm);

         if (!isInertiaUpToDate)
         {
            // First we evaluate M^-1 that is the inverse of the apparent inertia considering both bodies interacting in this contact.
            collisionCalculatorA.computeApparentLinearInertiaInverse(contactingBodyA, contactFrame, inverseApparentInertiaA);

            if (forwardDynamicsCalculatorB != null)
            {
               collisionCalculatorB.computeApparentLinearInertiaInverse(contactingBodyB, contactFrame, inverseApparentInertiaB);
               CommonOps.add(inverseApparentInertiaA, inverseApparentInertiaB, M_inv);
            }
            else
            {
               M_inv.set(inverseApparentInertiaA);
            }

            isInertiaUpToDate = true;
         }

         contactVelocity.get(c);
         LinearSolver<DenseMatrix64F> solver;
         if (CommonOps.det(M_inv) > 1.0e-6)
            solver = linearSolver;
         else
            solver = svdSolver; // TODO This is not enough to cover the degenerate case.

         M_inv_solverCopy.set(M_inv);
         solver.setA(M_inv_solverCopy);
         solver.solve(c, lambda_v_0);
         CommonOps.changeSign(lambda_v_0);

         if (lambda_v_0.get(2) > -1.0e-12 && ContactImpulseTools.isInsideFrictionCone(coefficientOfFriction, lambda_v_0))
         { // Contact is sticking, i.e. satisfies Coulomb's friction cone while canceling velocity.
            impulseA.getLinearPart().set(lambda_v_0);
         }
         else
         { // Contact is slipping, that's the though case.
            ContactImpulseTools.computeSlipLambda(beta1, beta2, beta3, gamma, coefficientOfFriction, M_inv, lambda_v_0, c, impulseA.getLinearPart(), false);
         }
      }
      else
      {
         collisionCalculatorA.reset();
         if (collisionCalculatorB != null)
            collisionCalculatorB.reset();
      }

      if (impulseA.getLinearPart().getZ() < 0.0)
         throw new IllegalStateException("Malformed impulse");

      if (isFirstUpdate)
      {
         impulseChangeA.set(impulseA.getLinearPart());
      }
      else
      {
         impulseA.getLinearPart().interpolate(previousImpulseA, impulseA.getLinearPart(), alpha);
         impulseChangeA.sub(impulseA.getLinearPart(), previousImpulseA);
      }

      if (forwardDynamicsCalculatorB != null)
      {
         impulseB.setIncludingFrame(contactingBodyB.getBodyFixedFrame(), impulseA);
         impulseB.negate();
      }

      previousImpulseA.set(impulseA.getLinearPart());
   }

   public double getImpulseUpdate()
   {
      return impulseChangeA.length();
   }

   public double getVelocityUpdate()
   {
      return contactVelocityChange.length();
   }

   public boolean isContactClosing()
   {
      return isContactClosing;
   }

   public void applyImpulseLazy()
   {
      if (!isContactClosing)
         return;

      collisionCalculatorA.applyImpulse(contactingBodyA, impulseA);

      if (collisionCalculatorB != null)
         collisionCalculatorB.applyImpulse(contactingBodyB, impulseB);
   }

   public double getDT()
   {
      return dt;
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

   public DenseMatrix64F computeJointVelocityChangeA()
   {
      if (!isContactClosing)
         return null;

      jointVelocityChangeA.set(collisionCalculatorA.applyAndPropagateImpulse(contactingBodyA, impulseA));
      return jointVelocityChangeA;
   }

   public DenseMatrix64F getJointVelocityChangeA()
   {
      return jointVelocityChangeA;
   }

   public RigidBodyBasics getContactingBodyB()
   {
      return contactingBodyB;
   }

   public ForwardDynamicsCalculator getForwardDynamicsCalculatorB()
   {
      return forwardDynamicsCalculatorB;
   }

   public DenseMatrix64F computeJointVelocityChangeB()
   {
      if (collisionCalculatorB == null)
         return null;
      if (!isContactClosing)
         return null;

      jointVelocityChangeB.set(collisionCalculatorB.applyAndPropagateImpulse(contactingBodyB, impulseB));
      return jointVelocityChangeB;
   }

   public DenseMatrix64F getJointVelocityChangeB()
   {
      if (collisionCalculatorB == null)
         return null;
      return jointVelocityChangeB;
   }

   public RigidBodyTwistProvider getTwistChangeProviderA()
   {
      return collisionCalculatorA.getTwistChangeProvider();
   }

   public RigidBodyTwistProvider getTwistChangeProviderB()
   {
      if (collisionCalculatorB != null)
         return collisionCalculatorB.getTwistChangeProvider();
      else
         return null;
   }

   public SpatialImpulseReadOnly getContactImpulseA()
   {
      return impulseA;
   }

   public SpatialImpulseReadOnly getContactImpulseB()
   {
      return impulseB;
   }

   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction;
   }

   public void printForUnitTest()
   {
      System.err.println(ContactImpulseTools.toStringForUnitTest(beta1, beta2, beta3, gamma, coefficientOfFriction, M_inv, lambda_v_0, c));
   }
}
