package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.stateMachines.State;


public abstract class AbstractFootControlState extends State<ConstraintType>
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   protected final YoVariableRegistry registry;

   // magic numbers:
   protected static final double minJacobianDeterminant = 0.035;
   protected static final double EPSILON_POINT_ON_EDGE = 1e-2;

   protected final ContactablePlaneBody contactableBody;
   protected final RigidBody rootBody;

   protected final FramePoint desiredPosition = new FramePoint(worldFrame);
   protected final FrameVector desiredLinearVelocity = new FrameVector(worldFrame);
   protected final FrameVector desiredLinearAcceleration = new FrameVector(worldFrame);
   protected final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
   protected final FrameVector desiredAngularAcceleration = new FrameVector(worldFrame);
   protected final FrameOrientation desiredOrientation = new FrameOrientation(worldFrame);
   protected final FrameOrientation trajectoryOrientation = new FrameOrientation(worldFrame);
   protected final SpatialAccelerationVector footAcceleration = new SpatialAccelerationVector();

   protected final RigidBodySpatialAccelerationControlModule accelerationControlModule;
   protected final MomentumBasedController momentumBasedController;
   protected final TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
   protected final int jacobianId;

   protected final DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(0, 1);
   protected final DoubleYoVariable nullspaceMultiplier;
   protected final GeometricJacobian jacobian;
   protected final BooleanYoVariable jacobianDeterminantInRange;
   protected final BooleanYoVariable doSingularityEscape;
   protected final DenseMatrix64F selectionMatrix;
   protected boolean isCoPOnEdge;
   protected FrameLineSegment2d edgeToRotateAbout;

   public AbstractFootControlState(ConstraintType stateEnum, RigidBodySpatialAccelerationControlModule accelerationControlModule,
         MomentumBasedController momentumBasedController, ContactablePlaneBody contactableBody, int jacobianId, DoubleYoVariable nullspaceMultiplier,
         BooleanYoVariable jacobianDeterminantInRange, BooleanYoVariable doSingularityEscape, RobotSide robotSide, YoVariableRegistry registry)
   {
      super(stateEnum);

      this.registry = registry;
      this.contactableBody = contactableBody;

      this.accelerationControlModule = accelerationControlModule;
      this.momentumBasedController = momentumBasedController;
      this.jacobianId = jacobianId;

      this.nullspaceMultiplier = nullspaceMultiplier;
      this.jacobianDeterminantInRange = jacobianDeterminantInRange;
      this.doSingularityEscape = doSingularityEscape;

      edgeToRotateAbout = new FrameLineSegment2d(contactableBody.getSoleFrame());
      rootBody = momentumBasedController.getTwistCalculator().getRootBody();
      taskspaceConstraintData.set(rootBody, contactableBody.getRigidBody());
      jacobian = momentumBasedController.getJacobian(jacobianId);

      selectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
   }

   public abstract void doSpecificAction();

   @Override
   public void doAction()
   {
      computeNullspaceMultipliers();
      doSpecificAction();
   }

   protected void setTaskspaceConstraint(SpatialAccelerationVector footAcceleration)
   {
      ReferenceFrame bodyFixedFrame = contactableBody.getRigidBody().getBodyFixedFrame();
      footAcceleration.changeBodyFrameNoRelativeAcceleration(bodyFixedFrame);
      footAcceleration.changeFrameNoRelativeMotion(bodyFixedFrame);
      taskspaceConstraintData.set(footAcceleration, nullspaceMultipliers, selectionMatrix);
      momentumBasedController.setDesiredSpatialAcceleration(jacobianId, taskspaceConstraintData);
   }

   private void computeNullspaceMultipliers()
   {
      double det = jacobian.det();
      jacobianDeterminantInRange.set(Math.abs(det) < minJacobianDeterminant);

      if (jacobianDeterminantInRange.getBooleanValue())
      {
         nullspaceMultipliers.reshape(1, 1);
         if (doSingularityEscape.getBooleanValue())
         {
            nullspaceMultipliers.set(0, nullspaceMultiplier.getDoubleValue());
         }
         else
         {
            nullspaceMultipliers.set(0, 0);
         }
      }
      else
      {
         nullspaceMultiplier.set(Double.NaN);
         nullspaceMultipliers.reshape(0, 1);
         doSingularityEscape.set(false);
      }
   }

   private final FrameConvexPolygon2d contactPolygon = new FrameConvexPolygon2d();
   private final FrameOrientation currentOrientation = new FrameOrientation();

   protected void determineCoPOnEdge()
   {
      FramePoint2d cop = momentumBasedController.getCoP(contactableBody);

      if (cop == null)
      {
         isCoPOnEdge = false;
      }
      else
      {
         List<FramePoint2d> contactPoints = contactableBody.getContactPoints2d();
         contactPolygon.setIncludingFrameAndUpdate(contactPoints);
         cop.changeFrame(contactPolygon.getReferenceFrame());
         FrameLineSegment2d closestEdge = contactPolygon.getClosestEdge(cop);
         boolean copOnEdge = closestEdge.distance(cop) < EPSILON_POINT_ON_EDGE;
         boolean hasCoPBeenOnEdge = isCoPOnEdge;
         if (copOnEdge && !hasCoPBeenOnEdge)
         {
            currentOrientation.setToZero(contactableBody.getFrameAfterParentJoint());
            currentOrientation.changeFrame(worldFrame);
         }
         isCoPOnEdge = copOnEdge;

         edgeToRotateAbout = closestEdge;
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      accelerationControlModule.reset();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      accelerationControlModule.reset();
   }
}
