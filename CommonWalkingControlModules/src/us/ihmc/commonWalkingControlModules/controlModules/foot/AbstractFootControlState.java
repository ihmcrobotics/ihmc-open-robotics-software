package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;

public abstract class AbstractFootControlState extends State<ConstraintType>
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final FootControlHelper footControlHelper;

   protected final RobotSide robotSide;
   protected final RigidBody rootBody;
   protected final ContactablePlaneBody contactableFoot;

   protected final FramePoint desiredPosition = new FramePoint(worldFrame);
   protected final FrameVector desiredLinearVelocity = new FrameVector(worldFrame);
   protected final FrameVector desiredLinearAcceleration = new FrameVector(worldFrame);
   protected final FrameOrientation desiredOrientation = new FrameOrientation(worldFrame);
   protected final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
   protected final FrameVector desiredAngularAcceleration = new FrameVector(worldFrame);
   protected final SpatialAccelerationVector footAcceleration = new SpatialAccelerationVector();

   protected final MomentumBasedController momentumBasedController;

   public AbstractFootControlState(ConstraintType stateEnum, FootControlHelper footControlHelper, YoVariableRegistry registry)
   {
      super(stateEnum);

      this.footControlHelper = footControlHelper;
      this.contactableFoot = footControlHelper.getContactableFoot();

      this.momentumBasedController = footControlHelper.getMomentumBasedController();

      this.robotSide = footControlHelper.getRobotSide();

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
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }
}
