package us.ihmc.rdx.ui.teleoperation.locomotion;

import controller_msgs.VelocityBasedWalkingInputMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

/**
 * Widgets for sending velocity-based walk commands to the RL walking controller, drawn at the top of the
 * teleoperation Locomotion header. Publishes {@link VelocityBasedWalkingInputMessage} via
 * {@link ROS2ControllerHelper#publishToController} at 10 Hz while walk is enabled, and sends a stop command
 * whenever walk is disabled so the RL command heartbeat does not expire mid-stride.
 */
public class RDXRLWalkingCommandWidgets
{
   private static final double PUBLISH_HZ = 10.0;

   private static final float MAX_FORWARD_VELOCITY = 0.8f;  // m/s  (matches RLDesireds defaults)
   private static final float MAX_LATERAL_VELOCITY  = 0.5f;  // m/s
   private static final float MAX_TURN_VELOCITY     = 1.0f;  // rad/s

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final ROS2ControllerHelper controllerHelper;
   private final VelocityBasedWalkingInputMessage message = new VelocityBasedWalkingInputMessage();
   private final Throttler throttler = new Throttler();

   private final ImBoolean walk           = new ImBoolean(false);
   private final ImFloat  forwardVelocity = new ImFloat(0.0f);
   private final ImFloat  lateralVelocity = new ImFloat(0.0f);
   private final ImFloat  turnVelocity    = new ImFloat(0.0f);

   private boolean wasWalking = false;

   public RDXRLWalkingCommandWidgets(ROS2ControllerHelper controllerHelper)
   {
      this.controllerHelper = controllerHelper;
      throttler.setPeriod(1.0 / PUBLISH_HZ);
   }

   public void update()
   {
      boolean isWalking = walk.get();

      // Send a zero-velocity stop command whenever walk is toggled off
      if (wasWalking && !isWalking)
         publishStop();

      wasWalking = isWalking;

      if (isWalking && throttler.run())
         publishCurrent();
   }

   /**
    * @param widgetStartX the X the widgets start at, past the labels, to line up with the rest of the header
    */
   public void renderImGuiWidgets(float widgetStartX)
   {
      ImGui.text("RL Walking:");
      ImGui.sameLine();
      ImGui.setCursorPosX(widgetStartX);
      ImGui.checkbox(labels.get("Walk"), walk);
      ImGui.sameLine();
      if (ImGui.button(labels.get("Stop")))
      {
         walk.set(false);
         forwardVelocity.set(0.0f);
         lateralVelocity.set(0.0f);
         turnVelocity.set(0.0f);
         publishStop();
      }

      renderVelocitySlider("Forward:", "Forward", forwardVelocity, MAX_FORWARD_VELOCITY, "%.2f m/s", widgetStartX);
      renderVelocitySlider("Lateral:", "Lateral", lateralVelocity, MAX_LATERAL_VELOCITY, "%.2f m/s", widgetStartX);
      renderVelocitySlider("Turn:", "Turn", turnVelocity, MAX_TURN_VELOCITY, "%.2f rad/s", widgetStartX);

      ImGui.setCursorPosX(widgetStartX);
      ImGui.textDisabled("Velocities are sent in m/s (not normalized).");
      ImGui.setCursorPosX(widgetStartX);
      ImGui.textDisabled("Check 'Walk' to start sending commands.");
   }

   private void renderVelocitySlider(String text, String id, ImFloat velocity, float maxVelocity, String format, float widgetStartX)
   {
      ImGui.text(text);
      ImGui.sameLine();
      ImGui.setCursorPosX(widgetStartX);
      ImGui.setNextItemWidth(200.0f);
      ImGui.sliderFloat(labels.getHidden(id), velocity.getData(), -maxVelocity, maxVelocity, format);
   }

   private void publishCurrent()
   {
      message.setWalk(true);
      message.setForwardVelocity(forwardVelocity.get());
      message.setLateralVelocity(lateralVelocity.get());
      message.setTurnVelocity(turnVelocity.get());
      message.setAreVelocitiesNormalized(false);
      controllerHelper.publishToController(message);
   }

   private void publishStop()
   {
      message.setWalk(false);
      message.setForwardVelocity(0.0);
      message.setLateralVelocity(0.0);
      message.setTurnVelocity(0.0);
      message.setAreVelocitiesNormalized(false);
      controllerHelper.publishToController(message);
   }
}
