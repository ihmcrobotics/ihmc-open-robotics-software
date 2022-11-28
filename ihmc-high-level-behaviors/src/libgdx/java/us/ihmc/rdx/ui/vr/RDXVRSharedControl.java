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
   private ImBoolean enabledRecording;
   private ImBoolean enabledIKStreaming;
   private final ImBoolean enabled = new ImBoolean(false);
   private final ProMPAssistant proMPAssistant = new ProMPAssistant();

   public RDXVRSharedControl(ImBoolean enabledIKStreaming, ImBoolean enabledReplay, ImBoolean enabledRecording)
   {
      this.enabledIKStreaming = enabledIKStreaming;
      this.enabledReplay = enabledReplay;
      this.enabledRecording = enabledRecording;
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
      ImGui.text("Toggle shared control assistance: Left B button");
      if (ImGui.checkbox(labels.get("Shared Control"), enabled))
      {
         setEnabled(enabled.get());
      }
   }

   private void setEnabled(boolean enabled)
   {
      if (enabled != this.enabled.get()){
         this.enabled.set(enabled);
         if (enabled == false){
            proMPAssistant.reset();
            enabledIKStreaming.set(false);
         }
      }
      if (enabled)
      {
         if (enabledRecording.get() || enabledReplay.get())
            this.enabled.set(false); //check no concurrency with recording and replay
      }
   }

   public boolean isActive()
   {
      return this.enabled.get();
   }
}
