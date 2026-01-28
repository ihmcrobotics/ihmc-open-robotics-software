package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.HandContactMessage;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandContactCommand;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class StandingPushRecoveryPlugin implements HighLevelHumanoidControllerPlugin
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final CommandInputManager commandInputManager;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final YoBoolean isHandRecoveryContactEnabled = new YoBoolean("isHandRecoveryContactEnabled", registry);
   private final YoDouble handTrajectoryTime = new YoDouble("handTrajectoryTime", registry);
   private final YoBoolean isFalling = new YoBoolean("isFalling", registry);
   private final YoBoolean sendHandContactMessage = new YoBoolean("sendHandContactMessage", registry);
   private boolean hasSentHandTrajectory = false;

   public StandingPushRecoveryPlugin(CommandInputManager commandInputManager, HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this.commandInputManager = commandInputManager;
      this.controllerToolbox = controllerToolbox;
      handTrajectoryTime.set(0.2);

      isHandRecoveryContactEnabled.set(true);
      controllerToolbox.getYoVariableRegistry().addChild(registry);
   }

   @Override
   public void update(double time)
   {
      double comVelocityX = controllerToolbox.getCenterOfMassVelocity().getX();
      double comVelocityY = controllerToolbox.getCenterOfMassVelocity().getY();
      isFalling.set(EuclidCoreTools.norm(comVelocityX, comVelocityY) > 0.07); // TODO make this better

      if (!hasSentHandTrajectory && isFalling.getValue() && isHandRecoveryContactEnabled.getValue())
      {
         hasSentHandTrajectory = true;
         sendHandContactMessage.set(false);

         RobotSide robotSide = RobotSide.LEFT;
         Point3D point = new Point3D(1.041, -0.499, 1.300);
         Vector3D normal = new Vector3D(-0.887, 0.413, 0.208);

         // Send hand trajectory message
//         MovingReferenceFrame handControlFrame = controllerToolbox.getFullRobotModel().getHandControlFrame(robotSide);
//         FrameQuaternion handOrientation = new FrameQuaternion(handControlFrame);
//         handOrientation.changeFrame(ReferenceFrame.getWorldFrame());
//         HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(robotSide,
//                                                                                                        handTrajectoryTime.getValue(),
//                                                                                                        point,
//                                                                                                        handOrientation,
//                                                                                                        ReferenceFrame.getWorldFrame());
//         HandTrajectoryCommand handTrajectoryCommand = new HandTrajectoryCommand();
//         handTrajectoryCommand.setFromMessage(handTrajectoryMessage);
//
//         commandInputManager.submitCommand(handTrajectoryCommand);
//
//         // Send hand load-bearing message
//         HandLoadBearingMessage handLoadBearingMessage = new HandLoadBearingMessage();
//         handLoadBearingMessage.setCoefficientOfFriction(0.4);
//         handLoadBearingMessage.setRobotSide(robotSide.toByte());
//         handLoadBearingMessage.getContactNormalInWorld().set(normal);
//         handLoadBearingMessage.setLoad(true);
//
//         FramePoint3D contactPoint = new FramePoint3D(controllerToolbox.getFullRobotModel().getHandControlFrame(robotSide));
//         contactPoint.changeFrame(controllerToolbox.getFullRobotModel().getHand(robotSide).getBodyFixedFrame());
//         handLoadBearingMessage.getContactPointInBodyFrame().set(contactPoint);
//
//         HandLoadBearingCommand handLoadBearingCommand = new HandLoadBearingCommand();
//         handLoadBearingCommand.setFromMessage(handLoadBearingMessage);
//         handLoadBearingCommand.setExecutionDelayTime(handTrajectoryTime.getValue());
//         commandInputManager.submitCommand(handLoadBearingCommand);

         HandContactMessage handContactMessage = new HandContactMessage();
         handContactMessage.setRobotSide(robotSide.toByte());
         handContactMessage.setTrajectoryDuration(0.24);
         handContactMessage.getBracingPoint().set(point);
         handContactMessage.getBracingNormal().set(normal);

         HandContactCommand reactiveBracingCommand = new HandContactCommand();
         reactiveBracingCommand.setFromMessage(handContactMessage);
         commandInputManager.submitCommand(reactiveBracingCommand);
      }
   }
}
