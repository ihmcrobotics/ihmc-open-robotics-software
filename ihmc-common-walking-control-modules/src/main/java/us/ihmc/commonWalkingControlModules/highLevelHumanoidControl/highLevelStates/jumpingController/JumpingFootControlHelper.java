package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.*;
import us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold.FootholdRotationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class JumpingFootControlHelper
{
   private final RobotSide robotSide;
   private final ContactableFoot contactableFoot;
   private final JumpingControllerToolbox controllerToolbox;
   private final WalkingControllerParameters walkingControllerParameters;

   private final FrameVector3D fullyConstrainedNormalContactVector;

   public JumpingFootControlHelper(RobotSide robotSide,
                                   WalkingControllerParameters walkingControllerParameters,
                                   JumpingControllerToolbox controllerToolbox)
   {
      this.robotSide = robotSide;
      this.controllerToolbox = controllerToolbox;
      this.walkingControllerParameters = walkingControllerParameters;

      contactableFoot = controllerToolbox.getContactableFeet().get(robotSide);
      RigidBodyBasics foot = contactableFoot.getRigidBody();

      fullyConstrainedNormalContactVector = new FrameVector3D(contactableFoot.getSoleFrame(), 0.0, 0.0, 1.0);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public ContactableFoot getContactableFoot()
   {
      return contactableFoot;
   }

   public JumpingControllerToolbox getJumpingControllerToolbox()
   {
      return controllerToolbox;
   }

   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   public SwingTrajectoryParameters getSwingTrajectoryParameters()
   {
      return walkingControllerParameters.getSwingTrajectoryParameters();
   }

   public void setFullyConstrainedNormalContactVector(FrameVector3D normalContactVector)
   {
      if (normalContactVector != null)
         fullyConstrainedNormalContactVector.setIncludingFrame(normalContactVector);
      else
         fullyConstrainedNormalContactVector.setIncludingFrame(contactableFoot.getSoleFrame(), 0.0, 0.0, 1.0);
   }
}
