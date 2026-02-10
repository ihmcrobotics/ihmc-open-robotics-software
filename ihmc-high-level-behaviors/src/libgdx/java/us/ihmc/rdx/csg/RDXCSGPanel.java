package us.ihmc.rdx.csg;

import controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage;
import controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage;
import controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.CSGROS2CommunicationHelper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.ros2.ROS2Node;

public class RDXCSGPanel extends RDXPanel
{
   private static final boolean PUBLISH_IN_VELOCITY_UNITS = true;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final ImBoolean requestWalkCSG = new ImBoolean();

   private final ImDouble forwardVelocity = new ImDouble();
   private final ImDouble lateralVelocity = new ImDouble();
   private final ImDouble turningVelocity = new ImDouble();

   private final ImDouble maxStepLengthForwards = new ImDouble();
   private final ImDouble maxStepLengthBackwards = new ImDouble();
   private final ImDouble maxStepWidth = new ImDouble();
   private final ImDouble swingHeight = new ImDouble();

   private final ImDouble swingDuration = new ImDouble();
   private final ImDouble transferDuration = new ImDouble();
   private final ImBoolean stepsAreAdjustable = new ImBoolean();
   private final ImBoolean snapToHeightMap = new ImBoolean();
   private final ImBoolean accountForGroundDrift = new ImBoolean();

   // CSG ROS communication helper
   private final CSGROS2CommunicationHelper controllerHelper;

   // CSG command and status messages
   private final ContinuousStepGeneratorInputMessage csgInputCommand = new ContinuousStepGeneratorInputMessage();
   private final ContinuousStepGeneratorParametersMessage csgParametersCommand;
   private final ContinuousStepGeneratorStatusMessage csgStatusMessage;

   public RDXCSGPanel(CSGROS2CommunicationHelper controllerHelper)
   {
      super("CSG Controls");
      super.setRenderMethod(this::renderImGuiWidgets);

      this.controllerHelper = controllerHelper;
      controllerHelper.addVolatileCSGStatusCallbackSubscription(this::reset);

      csgParametersCommand = controllerHelper.getCSGParametersCommand();
      csgStatusMessage = controllerHelper.getCSGStatusMessage();
   }

   public void renderImGuiWidgets()
   {
      // Calculate min and max velocities based on max step length and desired step duration
      double maxVelocityX;
      double minVelocityX;
      double minMaxVelocityY;
      double minMaxVelocityTurn;
      String velocityUnitsDescription;

      if (PUBLISH_IN_VELOCITY_UNITS)
      {
         csgInputCommand.setUnitVelocities(true);

         double currentStepTime = csgStatusMessage.getCurrentSwingDuration() + csgStatusMessage.getCurrentTransferDuration();
         double currentMaxStepLengthForwards = csgStatusMessage.getCurrentMaxStepLengthForwards();
         double currentMaxStepLengthBackwards = csgStatusMessage.getCurrentMaxStepLengthBackwards();
         double currentMaxStepWidth = csgStatusMessage.getCurrentMaxStepWidth();
         double currentTurnMaxAngleInward = csgStatusMessage.getCurrentTurnMaxAngleInward();
         double currentTurnMaxAngleOutward = csgStatusMessage.getCurrentTurnMaxAngleOutward();

         maxVelocityX = currentMaxStepLengthForwards / currentStepTime;
         minVelocityX = -currentMaxStepLengthBackwards / currentStepTime;
         minMaxVelocityY = currentMaxStepWidth / currentStepTime;
         minMaxVelocityTurn = (currentTurnMaxAngleOutward - currentTurnMaxAngleInward) / currentStepTime;

         velocityUnitsDescription = "(m/s)";
      }
      else
      {
         csgInputCommand.setUnitVelocities(false);

         maxVelocityX = 1.0;
         minVelocityX = -1.0;
         minMaxVelocityY = 1.0;
         minMaxVelocityTurn = 1.0;

         velocityUnitsDescription = "(% min/max)";
      }

      // Get user input data from GUI widgets
      boolean requestCSGWalkingChanged = ImGui.checkbox(labels.get("Request Walk CSG"), requestWalkCSG);

      boolean desiredForwardVelocityChanged = ImGuiTools.sliderDouble(labels.getHidden("Forward Velocity " + velocityUnitsDescription), forwardVelocity, minVelocityX, maxVelocityX, "Forward Velocity " + velocityUnitsDescription + ": %.2f");
      boolean desiredForwardVelocityIsHovered = ImGui.isItemHovered();

      boolean desiredLateralVelocityChanged = ImGuiTools.sliderDouble(labels.getHidden("Lateral Velocity " + velocityUnitsDescription), lateralVelocity, -minMaxVelocityY, minMaxVelocityY, "Lateral Velocity " + velocityUnitsDescription + ": %.2f");
      boolean desiredLateralVelocityIsHovered = ImGui.isItemHovered();

      boolean desiredTurningVelocityChanged = ImGuiTools.sliderDouble(labels.getHidden("Turning Velocity " + velocityUnitsDescription), turningVelocity, -minMaxVelocityTurn, minMaxVelocityTurn, "Turning Velocity " + velocityUnitsDescription + ": %.2f");
      boolean desiredTurningVelocityIsHovered = ImGui.isItemHovered();

      boolean swingDurationChanged = ImGuiTools.volatileInputDouble(labels.get("Swing Duration"), swingDuration);
      boolean transferDurationChanged = ImGuiTools.volatileInputDouble(labels.get("Transfer Duration"), transferDuration);

      boolean maxStepLengthForwardsChanged = ImGuiTools.volatileInputDouble(labels.get("Max Step Length Forwards"), maxStepLengthForwards);
      boolean maxStepLengthBackwardsChanged = ImGuiTools.volatileInputDouble(labels.get("Max Step Length Backwards"), maxStepLengthBackwards);
      boolean maxStepWidthChanged = ImGuiTools.volatileInputDouble(labels.get("Max Step Width"), maxStepWidth);

      boolean swingHeightChanged = ImGuiTools.volatileInputDouble(labels.get("Swing Height"), swingHeight);

      boolean stepsAreAdjustableChanged = ImGui.checkbox(labels.get("Steps Are Adjustable"), stepsAreAdjustable);
      boolean snapToHeightMapChanged = ImGui.checkbox(labels.get("Snap to Heightmap"), snapToHeightMap);
      boolean accountForGroundDriftChanged = ImGui.checkbox(labels.get("Account for Ground Drift"), accountForGroundDrift);

      if (requestCSGWalkingChanged)
         csgInputCommand.setWalk(requestWalkCSG.get());

      if (desiredForwardVelocityChanged)
         csgInputCommand.setForwardVelocity(forwardVelocity.get());

      if (desiredLateralVelocityChanged)
         csgInputCommand.setLateralVelocity(lateralVelocity.get());

      if (desiredTurningVelocityChanged)
         csgInputCommand.setTurnVelocity(turningVelocity.get());

      if (swingDurationChanged)
         csgParametersCommand.setSwingDuration(swingDuration.get());

      if (transferDurationChanged)
         csgParametersCommand.setTransferDuration(transferDuration.get());

      if (maxStepLengthForwardsChanged)
         csgParametersCommand.setMaxStepLengthForwards(maxStepLengthForwards.get());

      if (maxStepLengthBackwardsChanged)
         csgParametersCommand.setMaxStepLengthBackwards(maxStepLengthBackwards.get());

      if (maxStepWidthChanged)
         csgParametersCommand.setMaxStepWidth(maxStepWidth.get());

      if (swingHeightChanged)
         csgParametersCommand.setSwingHeight(swingHeight.get());

      if (stepsAreAdjustableChanged)
         csgParametersCommand.setStepsAreAdjustable(stepsAreAdjustable.get());

      if (snapToHeightMapChanged)
         csgParametersCommand.setSnapToHeightmap(snapToHeightMap.get());

      if (accountForGroundDriftChanged)
         csgParametersCommand.setAccountForGroundDrift(accountForGroundDrift.get());


      boolean publishCSGInputCommand = requestCSGWalkingChanged || desiredForwardVelocityChanged || desiredLateralVelocityChanged
                               || desiredTurningVelocityChanged;

      boolean publishCSGParametersCommand =
            swingDurationChanged || transferDurationChanged || maxStepLengthForwardsChanged || maxStepLengthBackwardsChanged
            || maxStepWidthChanged || swingHeightChanged || stepsAreAdjustableChanged || snapToHeightMapChanged || accountForGroundDriftChanged;

      if (desiredForwardVelocityIsHovered && ImGui.isMouseReleased(ImGuiMouseButton.Left))
      {
         forwardVelocity.set(0.0);
         csgInputCommand.setForwardVelocity(0.0);
         controllerHelper.publish(csgInputCommand);
         publishCSGInputCommand = false;
      }

      if (desiredLateralVelocityIsHovered && ImGui.isMouseReleased(ImGuiMouseButton.Left))
      {
         lateralVelocity.set(0.0);
         csgInputCommand.setLateralVelocity(0.0);
         controllerHelper.publish(csgInputCommand);
         publishCSGInputCommand = false;
      }

      if (desiredTurningVelocityIsHovered && ImGui.isMouseReleased(ImGuiMouseButton.Left))
      {
         turningVelocity.set(0.0);
         csgInputCommand.setTurnVelocity(0.0);
         controllerHelper.publish(csgInputCommand);
         publishCSGInputCommand = false;
      }

      if (publishCSGInputCommand)
         controllerHelper.publishAtThrottledRate(csgInputCommand);

      if (publishCSGParametersCommand)
         controllerHelper.publishAtThrottledRate(csgParametersCommand);
   }

   private void reset(ContinuousStepGeneratorStatusMessage csgStatusMessage)
   {
      requestWalkCSG.set(csgStatusMessage.getIsWalking());
      forwardVelocity.set(csgStatusMessage.getCurrentForwardVelocity());
      lateralVelocity.set(csgStatusMessage.getCurrentLateralVelocity());
      turningVelocity.set(csgStatusMessage.getCurrentTurnVelocity());

      swingDuration.set(csgStatusMessage.getCurrentSwingDuration());
      transferDuration.set(csgStatusMessage.getCurrentTransferDuration());

      maxStepLengthForwards.set(csgStatusMessage.getCurrentMaxStepLengthForwards());
      maxStepLengthBackwards.set(csgStatusMessage.getCurrentMaxStepLengthBackwards());
      maxStepWidth.set(csgStatusMessage.getCurrentMaxStepWidth());

      swingHeight.set(csgStatusMessage.getCurrentSwingHeight());

      stepsAreAdjustable.set(csgStatusMessage.getAreStepsAdjustable());
      snapToHeightMap.set(csgStatusMessage.getSnappingToHeightmap());
      accountForGroundDrift.set(csgStatusMessage.getAccountingForGroundDrift());
   }
}
