package us.ihmc.commonWalkingControlModules.controlModules.foot;

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
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.stateMachines.State;

public abstract class AbstractFootControlState extends State<ConstraintType>
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final FootControlHelper footControlHelper;

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

   protected FrameLineSegment2d edgeToRotateAbout;
   protected final RobotSide robotSide;

   public AbstractFootControlState(ConstraintType stateEnum, FootControlHelper footControlHelper, YoVariableRegistry registry)
   {
      super(stateEnum);

      this.footControlHelper = footControlHelper;
      this.contactableBody = footControlHelper.getContactableFoot();

      this.accelerationControlModule = footControlHelper.getAccelerationControlModule();
      this.momentumBasedController = footControlHelper.getMomentumBasedController();

      this.robotSide = footControlHelper.getRobotSide();

      edgeToRotateAbout = new FrameLineSegment2d(contactableBody.getSoleFrame());
      rootBody = momentumBasedController.getTwistCalculator().getRootBody();
   }

   public abstract void doSpecificAction();

   @Override
   public void doAction()
   {
      doSpecificAction();
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
