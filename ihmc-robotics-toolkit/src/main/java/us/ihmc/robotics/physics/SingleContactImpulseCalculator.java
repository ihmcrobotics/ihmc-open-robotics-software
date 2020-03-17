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
public class SingleContactImpulseCalculator implements ImpulseBasedConstraintCalculator
{
   private double coefficientOfFriction = 0.7;
   private double beta1 = 0.35;
   private double beta2 = 0.95;
   private double beta3 = 1.15;
   private double gamma = 1.0e-6;
   private double springConstant = 5.0;

   private boolean isInitialized = false;
   private boolean isInertiaUpToDate = false;
   private boolean isImpulseZero = false;
   private boolean isContactClosing = false;

   private final ForwardDynamicsCalculator forwardDynamicsCalculatorA;
   private final ForwardDynamicsCalculator forwardDynamicsCalculatorB;

   private final MultiBodyResponseCalculator responseCalculatorA;
   private final MultiBodyResponseCalculator responseCalculatorB;

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

   private double dt;

   private CollisionResult collisionResult;
   private RigidBodyTwistProvider externalRigidBodyTwistModifier;

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

      responseCalculatorA = new MultiBodyResponseCalculator(forwardDynamicsCalculatorA);

      if (forwardDynamicsCalculatorB != null)
         responseCalculatorB = new MultiBodyResponseCalculator(forwardDynamicsCalculatorB);
      else
         responseCalculatorB = null;
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

   @Override
   public void setExternalTwistModifier(RigidBodyTwistProvider externalRigidBodyTwistModifier)
   {
      this.externalRigidBodyTwistModifier = externalRigidBodyTwistModifier;
   }

   @Override
   public void reset()
   {
      isInitialized = false;
   }

   @Override
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

   private final FrameVector3D collisionPositionTerm = new FrameVector3D();

   @Override
   public void updateImpulse(double alpha)
   {
      boolean isFirstUpdate = !isInitialized;
      initialize();

      if (externalRigidBodyTwistModifier != null)
      {
         velocityDueToOtherImpulseA.setIncludingFrame(externalRigidBodyTwistModifier.getLinearVelocityOfBodyFixedPoint(contactingBodyA, contactPointA));
         velocityDueToOtherImpulseA.changeFrame(contactFrame);
         contactPointVelocityA.add(noImpulseVelocityA, velocityDueToOtherImpulseA);
      }
      else
      {
         contactPointVelocityA.set(noImpulseVelocityA);
      }

      if (rootB != null)
      {
         if (externalRigidBodyTwistModifier != null)
         {
            velocityDueToOtherImpulseB.setIncludingFrame(externalRigidBodyTwistModifier.getLinearVelocityOfBodyFixedPoint(contactingBodyB, contactPointB));
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
        // TODO Move the spring stuff to before the test to update isContactClosing
         collisionPositionTerm.setIncludingFrame(collisionResult.getPointOnBRootFrame());
         collisionPositionTerm.sub(collisionResult.getPointOnARootFrame());
         collisionPositionTerm.scale(springConstant);
         collisionPositionTerm.changeFrame(contactVelocity.getReferenceFrame());
         contactVelocity.sub(collisionPositionTerm);

         if (!isInertiaUpToDate)
         {
            // First we evaluate M^-1 that is the inverse of the apparent inertia considering both bodies interacting in this contact.
            responseCalculatorA.computeRigidBodyApparentLinearInertiaInverse(contactingBodyA, contactFrame, inverseApparentInertiaA);

            if (forwardDynamicsCalculatorB != null)
            {
               responseCalculatorB.computeRigidBodyApparentLinearInertiaInverse(contactingBodyB, contactFrame, inverseApparentInertiaB);
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

      if (impulseA.getLinearPart().getZ() < 0.0)
         throw new IllegalStateException("Malformed impulse");

      if (isFirstUpdate)
      {
         if (isContactClosing)
         {
            impulseChangeA.set(impulseA.getLinearPart());
            isImpulseZero = !isContactClosing;
         }
         else
         {
            impulseChangeA.setToZero(contactFrame);
            isImpulseZero = true;
         }
      }
      else
      {
         impulseA.getLinearPart().interpolate(previousImpulseA, impulseA.getLinearPart(), alpha);
         impulseChangeA.sub(impulseA.getLinearPart(), previousImpulseA);
         isImpulseZero = impulseA.getLinearPart().length() < 1.0e-10;
      }


      if (isImpulseZero)
      {
         responseCalculatorA.reset();

         if (responseCalculatorB != null)
         {
            impulseB.setToZero(contactingBodyB.getBodyFixedFrame(), impulseA.getReferenceFrame());
            responseCalculatorB.reset();
         }
      }
      else
      {
         responseCalculatorA.applyRigidBodyImpulse(contactingBodyA, impulseA);

         if (responseCalculatorB != null)
         {
            impulseB.setIncludingFrame(contactingBodyB.getBodyFixedFrame(), impulseA);
            impulseB.negate();
            responseCalculatorB.applyRigidBodyImpulse(contactingBodyB, impulseB);
         }
      }

      previousImpulseA.set(impulseA.getLinearPart());
   }

   @Override
   public double getImpulseUpdate()
   {
      return impulseChangeA.length();
   }

   @Override
   public double getVelocityUpdate()
   {
      return contactVelocityChange.length();
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

   @Override
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

   @Override
   public int getNumberOfRobotsInvolved()
   {
      return responseCalculatorB == null ? 1 : 2;
   }

   @Override
   public RigidBodyBasics getRootBody(int index)
   {
      return index == 0 ? getRootBodyA() : getRootBodyB();
   }

   @Override
   public RigidBodyTwistProvider getRigidBodyTwistChangeProvider(int index)
   {
      return index == 0 ? getTwistChangeProviderA() : getTwistChangeProviderB();
   }

   @Override
   public JointStateProvider getJointTwistChangeProvider(int index)
   {
      return index == 0 ? JointStateProvider.toJointTwistProvider(responseCalculatorA) : JointStateProvider.toJointTwistProvider(responseCalculatorB);
   }

   @Override
   public DenseMatrix64F getJointVelocityChange(int index)
   {
      return index == 0 ? getJointVelocityChangeA() : getJointVelocityChangeB();
   }

   public RigidBodyBasics getRootBodyA()
   {
      return rootA;
   }

   public RigidBodyBasics getRootBodyB()
   {
      return rootB;
   }

   public RigidBodyTwistProvider getTwistChangeProviderA()
   {
      return responseCalculatorA.getTwistChangeProvider();
   }

   public RigidBodyTwistProvider getTwistChangeProviderB()
   {
      if (responseCalculatorB != null)
         return responseCalculatorB.getTwistChangeProvider();
      else
         return null;
   }

   public DenseMatrix64F getJointVelocityChangeA()
   {
      return isImpulseZero ? null : responseCalculatorA.propagateImpulse();
   }

   public RigidBodyBasics getContactingBodyB()
   {
      return contactingBodyB;
   }

   public ForwardDynamicsCalculator getForwardDynamicsCalculatorB()
   {
      return forwardDynamicsCalculatorB;
   }

   public DenseMatrix64F getJointVelocityChangeB()
   {
      if (responseCalculatorB == null)
         return null;
      return isImpulseZero ? null : responseCalculatorB.propagateImpulse();
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
