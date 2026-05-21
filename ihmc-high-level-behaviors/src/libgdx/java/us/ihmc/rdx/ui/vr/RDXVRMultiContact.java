package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.HandLoadBearingMessage;
import imgui.type.ImBoolean;
import toolbox_msgs.HumanoidKinematicsToolboxConfigurationMessage;
import toolbox_msgs.KinematicsStreamingToolboxConfigurationMessage;
import toolbox_msgs.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.KinematicsToolboxCenterOfMassMessage;
import toolbox_msgs.KinematicsToolboxOutputStatus;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.rdx.ui.graphics.RDXMultiContactRegionGraphic;
import us.ihmc.rdx.ui.graphics.RDXMultiContactRegionHelper;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import static us.ihmc.communication.packets.MessageTools.toFrameId;

public class RDXVRMultiContact
{
   private static final double COM_CONTROL_JOYSTICK_THRESHOLD = 0.7;
   private static final double COM_JOYSTICK_INCREMENT = 6.0e-4;

   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final FullHumanoidRobotModel ghostFullRobotModel;
   private final double streamPeriod;
   private final RDXVRContext vrContext;
   private final ImBoolean userIsControllingRobot;

   private final RDXMultiContactRegionGraphic multiContactStabilityGraphic;
   private final RDXMultiContactRegionHelper regionHelper;
   private final HumanoidKinematicsToolboxConfigurationMessage ikHumanoidSolverConfigurationMessage = new HumanoidKinematicsToolboxConfigurationMessage();
   private final KinematicsStreamingToolboxConfigurationMessage streamingToolboxConfigurationMessage = new KinematicsStreamingToolboxConfigurationMessage();

   private final FramePoint3D comPositionInitial = new FramePoint3D();
   private final FrameVector3D comTrackerOffset = new FrameVector3D();
   private final FramePoint3D desiredCoMPositionFiltered = new FramePoint3D();
   private final FrameVector3D desiredCoMVelocity = new FrameVector3D();
   private final SideDependentList<Float> joystickLateralValue = new SideDependentList<>();
   private final SideDependentList<Float> joystickForwardValue = new SideDependentList<>();

   private final SideDependentList<RDXHandControlMode> handControlModes;
   private final SideDependentList<Boolean> handsAreLoaded = new SideDependentList<>(false, false);

   public RDXVRMultiContact(ROS2SyncedRobotModel syncedRobot,
                            FullHumanoidRobotModel ghostFullRobotModel,
                            ROS2ControllerHelper ros2ControllerHelper,
                            RDXVRContext vrContext,
                            double streamPeriod,
                            ImBoolean userIsControllingRobot,
                            SideDependentList<RDXHandControlMode> handControlModes)
   {
      this.syncedRobot = syncedRobot;
      this.ghostFullRobotModel = ghostFullRobotModel;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.vrContext = vrContext;
      this.streamPeriod = streamPeriod;
      this.handControlModes = handControlModes;
      this.userIsControllingRobot = userIsControllingRobot;

      multiContactStabilityGraphic = new RDXMultiContactRegionGraphic(ghostFullRobotModel);
      regionHelper = new RDXMultiContactRegionHelper(ghostFullRobotModel, ros2ControllerHelper.getROS2Node());
   }

   public void processVRInput()
   {
      if (!isEnabled())
         return;
      if (!userIsControllingRobot.get())
         return;

      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller ->
          {
             joystickForwardValue.put(side, controller.getJoystickActionData().y());
             joystickLateralValue.put(side, -controller.getJoystickActionData().x());

             if (handControlModes.get(side) == RDXHandControlMode.LOAD_BEARING)
             {
                handsAreLoaded.put(side, true);
                sendHandLoadBearingMessage(side);
             }
             else
             {
                handsAreLoaded.put(side, false);
             }
          });
      }
   }

   // When hands are loaded, allow the user to directly control the CoM with the right joysticks
   public void doCoMControl(KinematicsStreamingToolboxInputMessage toolboxInputMessage)
   {
      if (!isEnabled())
         return;

      boolean isUpperBodyLoadBearing = handsAreLoaded.get(RobotSide.LEFT) || handsAreLoaded.get(RobotSide.RIGHT);
      if (!isUpperBodyLoadBearing)
         return;

      comTrackerOffset.changeFrame(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
      if (Math.abs(joystickForwardValue.get(RobotSide.RIGHT)) > COM_CONTROL_JOYSTICK_THRESHOLD)
         comTrackerOffset.addX(Math.signum(joystickForwardValue.get(RobotSide.RIGHT)) * COM_JOYSTICK_INCREMENT);
      if (Math.abs(joystickLateralValue.get(RobotSide.RIGHT)) > COM_CONTROL_JOYSTICK_THRESHOLD)
         comTrackerOffset.addY(Math.signum(joystickLateralValue.get(RobotSide.RIGHT)) * COM_JOYSTICK_INCREMENT);
      if (Math.abs(joystickForwardValue.get(RobotSide.LEFT)) > COM_CONTROL_JOYSTICK_THRESHOLD)
         comTrackerOffset.addZ(Math.signum(joystickForwardValue.get(RobotSide.LEFT)) * COM_JOYSTICK_INCREMENT);
      comTrackerOffset.changeFrame(ReferenceFrame.getWorldFrame());

      FramePoint3D desiredCenterOfMass = new FramePoint3D(comPositionInitial);
      desiredCenterOfMass.changeFrame(ReferenceFrame.getWorldFrame());
      desiredCenterOfMass.add(comTrackerOffset);

      desiredCoMVelocity.setAndScale(-1.0, desiredCoMPositionFiltered);

      double interpolationAlpha = 0.06;
      desiredCoMPositionFiltered.setX(EuclidCoreTools.interpolate(desiredCoMPositionFiltered.getX(), desiredCenterOfMass.getX(), interpolationAlpha));
      desiredCoMPositionFiltered.setY(EuclidCoreTools.interpolate(desiredCoMPositionFiltered.getY(), desiredCenterOfMass.getY(), interpolationAlpha));
      desiredCoMPositionFiltered.setZ(EuclidCoreTools.interpolate(desiredCoMPositionFiltered.getZ(), desiredCenterOfMass.getZ(), interpolationAlpha));

      desiredCoMVelocity.add(desiredCoMPositionFiltered);
      desiredCoMVelocity.scale(1.0 / streamPeriod);

      KinematicsToolboxCenterOfMassMessage comMessage = new KinematicsToolboxCenterOfMassMessage();
      comMessage.getDesiredPositionInWorld().set(desiredCoMPositionFiltered);
      comMessage.getDesiredLinearVelocityInWorld().getVector().setToZero();

      comMessage.getSelectionMatrix().setSelectionFrameId(toFrameId(ReferenceFrame.getWorldFrame()));
      comMessage.getSelectionMatrix().setXSelected(true);
      comMessage.getSelectionMatrix().setYSelected(true);
      comMessage.getSelectionMatrix().setZSelected(true);

      comMessage.setHasDesiredLinearVelocity(true);
      comMessage.getDesiredLinearVelocityInWorld().set(desiredCoMVelocity);

      double comWeight = 2.0 / ghostFullRobotModel.getTotalMass();
      comMessage.getWeights().setXWeight(comWeight);
      comMessage.getWeights().setYWeight(comWeight);
      comMessage.getWeights().setZWeight(comWeight);

      toolboxInputMessage.setUseCenterOfMassInput(true);
      toolboxInputMessage.getCenterOfMassInput().set(comMessage);
   }

   public void update(KinematicsToolboxOutputStatus latestStatus)
   {
      if (isEnabled())
      {
         multiContactStabilityGraphic.update(latestStatus, desiredCoMPositionFiltered);
      }
   }

   private boolean isEnabled()
   {
      return handControlModes.get(RobotSide.LEFT) == RDXHandControlMode.LOAD_BEARING || handControlModes.get(RobotSide.RIGHT) == RDXHandControlMode.LOAD_BEARING;
   }

   public void reset()
   {
      if (!isEnabled())
         return;

      comPositionInitial.setToZero(syncedRobot.getReferenceFrames().getCenterOfMassFrame());
      comPositionInitial.changeFrame(ReferenceFrame.getWorldFrame());
      comTrackerOffset.setToZero();
      desiredCoMPositionFiltered.set(comPositionInitial);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (isEnabled())
      {
         multiContactStabilityGraphic.getRenderables(renderables, pool);
      }
   }

   private void sendHandLoadBearingMessage(RobotSide robotSide)
   {
      HandLoadBearingMessage handLoadBearingMessage = new HandLoadBearingMessage();
      handLoadBearingMessage.setRobotSide(robotSide.toByte());

      if (handsAreLoaded.get(robotSide))
      {
         handLoadBearingMessage.setLoad(false);
         handsAreLoaded.put(robotSide, false);

         ikHumanoidSolverConfigurationMessage.setHoldCurrentCenterOfMassXyPosition(false);
         ros2ControllerHelper.publish(ControllerAPI.getTopic(KinematicsStreamingToolboxModule.getInputTopic(syncedRobot.getRobotModel().getSimpleRobotName()),
                                                             HumanoidKinematicsToolboxConfigurationMessage.class), ikHumanoidSolverConfigurationMessage);
      }
      else
      {
         handLoadBearingMessage.setLoad(true);

         double handCoefficientOfFriction = 0.4;
         handLoadBearingMessage.setCoefficientOfFriction(handCoefficientOfFriction);

         // Contact point assumed to be at hand control frame and is using the nubs
         FramePoint3D contactPoint = new FramePoint3D(syncedRobot.getFullRobotModel().getHandControlFrame(robotSide));
         contactPoint.changeFrame(syncedRobot.getFullRobotModel().getHand(robotSide).getBodyFixedFrame());
         handLoadBearingMessage.getContactPointInBodyFrame().set(contactPoint);

         Vector3DReadOnly contactNormal = regionHelper.getNormalOfClosestRegion(robotSide);

         // option to fail if no region detected
         //         if (contactNormal == null)
         //         {
         //            // no region detected
         //            LogTools.info("No region detected");
         //            return;
         //         }

         // optional fallback
         if (contactNormal == null)
         { // fall back on hard-coded
            FrameVector3D fallbackNormal = new FrameVector3D(syncedRobot.getReferenceFrames().getMidFeetZUpFrame(), -1.0, 0.0, 0.0);
            fallbackNormal.changeFrame(ReferenceFrame.getWorldFrame());
            contactNormal = fallbackNormal;
         }

         handLoadBearingMessage.getContactNormalInWorld().set(contactNormal);
         handsAreLoaded.put(robotSide, true);
      }

      ros2ControllerHelper.publishToController(handLoadBearingMessage);

      ikHumanoidSolverConfigurationMessage.setHoldCurrentCenterOfMassXyPosition(false);
      ros2ControllerHelper.publish(ControllerAPI.getTopic(KinematicsStreamingToolboxModule.getInputTopic(syncedRobot.getRobotModel().getSimpleRobotName()),
                                                          HumanoidKinematicsToolboxConfigurationMessage.class), ikHumanoidSolverConfigurationMessage);

      boolean isUpperBodyLoadBearing = handsAreLoaded.get(RobotSide.LEFT) || handsAreLoaded.get(RobotSide.RIGHT);
      streamingToolboxConfigurationMessage.setEnableCenterOfMassControl(isUpperBodyLoadBearing);
      ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStreamingConfigurationTopic(syncedRobot.getRobotModel().getSimpleRobotName()),
                                   streamingToolboxConfigurationMessage);
   }
}