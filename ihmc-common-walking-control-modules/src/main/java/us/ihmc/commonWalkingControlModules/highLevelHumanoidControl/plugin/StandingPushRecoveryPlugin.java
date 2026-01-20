package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.HandLoadBearingMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.ReactiveBracingMessage;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandLoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ReactiveBracingCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.geometry.PlanarRegion;
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
//   private final YoBoolean isFalling = new YoBoolean("isFalling", registry);
   private final YoBoolean sendReactiveBracingMessage = new YoBoolean("sendReactiveBracingMessage", registry);
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
//      double comVelocityX = controllerToolbox.getCenterOfMassVelocity().getX();
//      double comVelocityY = controllerToolbox.getCenterOfMassVelocity().getY();
//      isFalling.set(EuclidCoreTools.norm(comVelocityX, comVelocityY) > 0.07);

//      if (!hasSentHandTrajectory && isFalling.getValue() && isHandRecoveryContactEnabled.getValue())
      if (!hasSentHandTrajectory && sendReactiveBracingMessage.getValue())
      {
         hasSentHandTrajectory = true;
         sendReactiveBracingMessage.set(false);

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

         ReactiveBracingMessage reactiveBracingMessage = new ReactiveBracingMessage();
         reactiveBracingMessage.setRobotSide(robotSide.toByte());
         reactiveBracingMessage.getBracingPoint().set(point);
         reactiveBracingMessage.getBracingNormal().set(normal);

         ReactiveBracingCommand reactiveBracingCommand = new ReactiveBracingCommand();
         reactiveBracingCommand.setFromMessage(reactiveBracingMessage);
         commandInputManager.submitCommand(reactiveBracingCommand);
      }
   }
}
