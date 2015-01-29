package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;

public class FootControlHelper
{
   private final RobotSide robotSide;
   private final ContactablePlaneBody contactableFoot;
   private final MomentumBasedController momentumBasedController;
   private final TwistCalculator twistCalculator;
   private final RigidBodySpatialAccelerationControlModule accelerationControlModule;
   private final WalkingControllerParameters walkingControllerParameters;

   public FootControlHelper(RobotSide robotSide, WalkingControllerParameters walkingControllerParameters, MomentumBasedController momentumBasedController, YoVariableRegistry registry)
   {
      this.robotSide = robotSide;
      this.momentumBasedController = momentumBasedController;
      this.walkingControllerParameters = walkingControllerParameters;

      contactableFoot = momentumBasedController.getContactableFeet().get(robotSide);
      twistCalculator = momentumBasedController.getTwistCalculator();
      
      RigidBody foot = contactableFoot.getRigidBody();
      String namePrefix = foot.getName();
      ReferenceFrame bodyFrame = contactableFoot.getFrameAfterParentJoint();
      double controlDT = momentumBasedController.getControlDT();

      accelerationControlModule = new RigidBodySpatialAccelerationControlModule(namePrefix, twistCalculator, foot, bodyFrame, controlDT, registry);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public ContactablePlaneBody getContactableFoot()
   {
      return contactableFoot;
   }

   public MomentumBasedController getMomentumBasedController()
   {
      return momentumBasedController;
   }

   public TwistCalculator getTwistCalculator()
   {
      return twistCalculator;
   }

   public RigidBodySpatialAccelerationControlModule getAccelerationControlModule()
   {
      return accelerationControlModule;
   }

   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }
}
