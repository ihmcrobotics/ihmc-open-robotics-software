package us.ihmc.rdx.ui.graphics.ros2.foundationPose;

import imgui.ImGui;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.perception.detections.foundationPose.SyncedFoundationPoseParameters;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;

public class RDXROS2FoundationPoseSettings
{
   private final IsaacROSFoundationPoseObject object;

   private final ROS2Publisher<Empty> resetRequestPublisher;

   private final SyncedFoundationPoseParameters parameters;

   private final ImGuiUniqueLabelMap labels;

   private final ImBooleanWrapper enabled;
   private final ImBooleanWrapper autoResetEnabled;
   private final ImDoubleWrapper resetDistance;

   public RDXROS2FoundationPoseSettings(ROS2Node ros2Node, ROS2PeerClockOffsetEstimator ros2PeerClockOffsetEstimator, IsaacROSFoundationPoseObject object)
   {
      this.object = object;

      resetRequestPublisher = ros2Node.createPublisher(object.topics.reset());

      CRDTInfo crdtInfo = new CRDTInfo(ROS2ActorDesignation.OPERATOR, ros2PeerClockOffsetEstimator);
      parameters = new SyncedFoundationPoseParameters(ros2Node, crdtInfo, object);

      labels = new ImGuiUniqueLabelMap(getClass());
      enabled = new ImBooleanWrapper(parameters.getEnabled()::getValue,
                                     parameters.getEnabled()::setValue,
                                     imBoolean -> ImGui.checkbox(labels.getHidden("enabled"), imBoolean));
      autoResetEnabled = new ImBooleanWrapper(parameters.getAutoResetEnabled()::getValue,
                                              parameters.getAutoResetEnabled()::setValue,
                                              imBoolean -> ImGui.checkbox(labels.getHidden("auto reset enabled"), imBoolean));
      resetDistance = new ImDoubleWrapper(parameters.getResetDistance()::getValue,
                                          parameters.getResetDistance()::setValue,
                                          imDouble -> ImGuiTools.volatileInputDouble(labels.getHidden("reset distance"), imDouble));
   }

   public void update()
   {
      parameters.update();
   }

   // TODO: Make this prettier
   public void renderImGuiWidgets()
   {
      ImGui.text(object.name());

      ImGui.sameLine();
      enabled.renderImGuiWidget();

      ImGui.sameLine();
      if (ImGui.button(labels.get("Reset")))
         resetRequestPublisher.publish(new Empty());

      ImGui.sameLine();
      autoResetEnabled.renderImGuiWidget();

      ImGui.sameLine();
      resetDistance.renderImGuiWidget();
   }

   public void destroy()
   {
      resetRequestPublisher.remove();
   }
}
