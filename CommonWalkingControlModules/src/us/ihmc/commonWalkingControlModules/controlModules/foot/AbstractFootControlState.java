package us.ihmc.commonWalkingControlModules.controlModules.foot;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.stateMachines.State;

public abstract class AbstractFootControlState extends State<ConstraintType>
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   protected final YoVariableRegistry registry;

   protected final FootControlHelper footControlHelper;

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

   protected final GeometricJacobian jacobian;
   protected final DenseMatrix64F selectionMatrix;
   protected FrameLineSegment2d edgeToRotateAbout;
   protected final RobotSide robotSide;

   public AbstractFootControlState(ConstraintType stateEnum, FootControlHelper footControlHelper, YoVariableRegistry registry)
   {
      super(stateEnum);

      this.footControlHelper = footControlHelper;
      this.registry = registry;
      this.contactableBody = footControlHelper.getContactableFoot();

      this.accelerationControlModule = footControlHelper.getAccelerationControlModule();
      this.momentumBasedController = footControlHelper.getMomentumBasedController();
      this.jacobianId = footControlHelper.getJacobianId();

      this.robotSide = footControlHelper.getRobotSide();

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
      footControlHelper.computeNullspaceMultipliers();
      doSpecificAction();
   }

   protected void setTaskspaceConstraint(SpatialAccelerationVector footAcceleration)
   {
      ReferenceFrame bodyFixedFrame = contactableBody.getRigidBody().getBodyFixedFrame();
      footAcceleration.changeBodyFrameNoRelativeAcceleration(bodyFixedFrame);
      footAcceleration.changeFrameNoRelativeMotion(bodyFixedFrame);
      taskspaceConstraintData.set(footAcceleration, footControlHelper.getNullspaceMultipliers(), selectionMatrix);
      momentumBasedController.setDesiredSpatialAcceleration(jacobianId, taskspaceConstraintData);
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
