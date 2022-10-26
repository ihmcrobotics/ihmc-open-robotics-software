package us.ihmc.rdx.ui.vr;

import imgui.ImGui;
import imgui.type.ImBoolean;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.avatar.sharedControl.ProMPAssistant;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXVRSharedControl
{
   private ImBoolean enabledKinematicsStreaming;
   private ImBoolean enabledRecording;
   private final ImBoolean enabled = new ImBoolean(false);
   private boolean isActive = false;
   private final ProMPAssistant proMPAssistant = new ProMPAssistant();

   public RDXVRSharedControl(ImBoolean enabledKinematicsStreaming, ImBoolean enabledRecording)
   {
      this.enabledKinematicsStreaming = enabledKinematicsStreaming;
      this.enabledRecording = enabledRecording;
   }

   public void processInput(InputDigitalActionData triggerButton)
   {
      // check streaming is on, recording is on and trigger button has been pressed once. if button is pressed again recording is stopped
      if (enabledKinematicsStreaming.get() && enabled.get() && triggerButton.bChanged() && !triggerButton.bState())
      {
         isActive = !isActive;
      }
   }

   public void processFrameInformation(Pose3DReadOnly observedPose, String bodyPart)
   {
      proMPAssistant.processFrameInformation(observedPose,bodyPart);
   }

   public boolean readyToPack()
   {
      return proMPAssistant.readyToPack();
   }

   public void framePoseToPack(FramePose3D framePose, String bodyPart)
   {
      proMPAssistant.framePoseToPack(framePose,bodyPart);
   }

   public void renderWidgets(ImGuiUniqueLabelMap labels)
   {
      ImGui.text("Toggle shared control assistance: Right B button");
      if (ImGui.checkbox(labels.get("Shared Control"), enabled))
      {
         setEnabled(enabled.get());
      }
   }

   private void setEnabled(boolean enabled)
   {
      if (enabled != this.enabled.get())
         this.enabled.set(enabled);
      if (enabled)
      {
         if (enabledRecording.get())
            this.enabled.set(false); //check no concurrency with recording
      }
   }

   public boolean isActive()
   {
      return this.isActive;
   }
}
