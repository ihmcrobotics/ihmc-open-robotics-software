package us.ihmc.rdx.ui.vr;

import imgui.ImGui;
import imgui.type.ImBoolean;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.behaviors.sharedControl.ProMPAssistant;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXVRSharedControl
{
   private ImBoolean enabledReplay;
   private ImBoolean enabledIKStreaming;
   private final ImBoolean enabled = new ImBoolean(false);
   private final ProMPAssistant proMPAssistant = new ProMPAssistant();

   public RDXVRSharedControl(ImBoolean enabledIKStreaming, ImBoolean enabledReplay)
   {
      this.enabledIKStreaming = enabledIKStreaming;
      this.enabledReplay = enabledReplay;
   }

   public void processInput(InputDigitalActionData triggerButton)
   {
      // enable if trigger button has been pressed once. if button is pressed again shared control is stopped
      if (triggerButton.bChanged() && !triggerButton.bState())
      {
         setEnabled(!enabled.get());
      }
   }

   public void processFrameInformation(Pose3DReadOnly observedPose, String bodyPart)
   {
      proMPAssistant.processFrameInformation(observedPose, bodyPart);
   }

   public boolean readyToPack()
   {
      return proMPAssistant.readyToPack();
   }

   public void framePoseToPack(FramePose3D framePose, String bodyPart)
   {
      if (proMPAssistant.isCurrentTaskDone()) //do not want the assistant to keep recomputing trajectories for the same task over and over
         setEnabled(false); //exit promp assistance when the current task is over, reactivate it in VR or UI when you want to use it again
      else
         proMPAssistant.framePoseToPack(framePose, bodyPart); //use promp assistance for shared control
   }

   public void renderWidgets(ImGuiUniqueLabelMap labels)
   {
      ImGui.text("Toggle shared control assistance: Left B button");
      if (ImGui.checkbox(labels.get("Shared Control"), enabled))
      {
         setEnabled(enabled.get());
      }
   }

   private void setEnabled(boolean enabled)
   {
      if (enabled != this.enabled.get())
      {
         this.enabled.set(enabled);
         if (enabled == false) // if deactivated
         {
            // reset promp assistance
            proMPAssistant.reset();
            proMPAssistant.setCurrentTaskDone(false);
            enabledIKStreaming.set(false); //stop the ik streaming so that you can reposition according to the robot state to avoid jumps in poses
         }
      }
      if (enabled)
      {
         if (enabledReplay.get())
            this.enabled.set(false); //check no concurrency with replay
      }
   }

   public boolean isActive()
   {
      return this.enabled.get();
   }
}
