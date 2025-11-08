package us.ihmc.rdx.ui.graphics.ros2.foundationPose;

import imgui.ImGui;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.perception.detections.foundationPose.SyncedIsaacROSFoundationPoseParameters;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;

public class RDXIsaacROSFoundationPoseSettings
{
   private final IsaacROSFoundationPoseObject object;

   private final ROS2Publisher<Empty> resetRequestPublisher;
   private final SyncedIsaacROSFoundationPoseParameters parameters;

   private final ImGuiUniqueLabelMap labels;
   private final ImBooleanWrapper enabled;
   private final ImBooleanWrapper autoResetEnabled;
   private final ImDoubleWrapper resetDistance;

   public RDXIsaacROSFoundationPoseSettings(ROS2Node ros2Node,
                                            ROS2PeerClockOffsetEstimator ros2ClockOffsetEstimator,
                                            IsaacROSFoundationPoseObject object)
   {
      this.object = object;

      resetRequestPublisher = ros2Node.createPublisher(object.topics.reset());

      CRDTInfo crdtInfo = new CRDTInfo(ROS2ActorDesignation.OPERATOR, ros2ClockOffsetEstimator);
      parameters = new SyncedIsaacROSFoundationPoseParameters(ros2Node, crdtInfo, object);

      labels = new ImGuiUniqueLabelMap(getClass());
      enabled = new ImBooleanWrapper(parameters.getEnabled()::getValue,
                                     parameters.getEnabled()::setValue,
                                     imBoolean -> ImGui.checkbox(labels.getHidden("enabled"), imBoolean));
      autoResetEnabled = new ImBooleanWrapper(parameters.getAutoResetEnabled()::getValue,
                                              parameters.getAutoResetEnabled()::setValue,
                                              imBoolean -> ImGui.checkbox(labels.getHidden("auto reset enabled"), imBoolean));
      resetDistance = new ImDoubleWrapper(parameters.getResetDistance()::getValue,
                                          parameters.getResetDistance()::setValue,
                                          imDouble -> ImGuiTools.volatileInputDouble(labels.getHidden("reset distance"), imDouble, 0.0, 0.0, "%05.2f"));
   }

   public SyncedIsaacROSFoundationPoseParameters getParameters()
   {
      return parameters;
   }

   public void update()
   {
      parameters.update();
   }

   public void renderAsTableRow()
   {
      if (ImGui.tableNextColumn()) // Enable
      {
         ImGui.setNextItemWidth(-1.0f);
         enabled.renderImGuiWidget();
         ImGui.setItemAllowOverlap();
      }

      if (ImGui.tableNextColumn()) // Object
      {
         ImGui.setNextItemWidth(-1.0f);
         ImGui.text(object.name());
      }

      if (ImGui.tableNextColumn()) // Reset button
      {
         ImGui.setNextItemWidth(-1.0f);
         if (ImGui.button(labels.get("Reset")))
            resetRequestPublisher.publish(new Empty());
         ImGui.setItemAllowOverlap();
      }

      if (ImGui.tableNextColumn()) // Enable auto reset
      {
         ImGui.setNextItemWidth(-1.0f);
         autoResetEnabled.renderImGuiWidget();
         ImGui.setItemAllowOverlap();
      }

      if (ImGui.tableNextColumn()) // Auto reset distance
      {
         ImGui.setNextItemWidth(-1.0f);
         resetDistance.renderImGuiWidget();
         ImGui.setItemAllowOverlap();
      }
   }

   public void destroy()
   {
      resetRequestPublisher.remove();
   }
}
