package us.ihmc.rdx.ui.graphics.ros2.supervisePose;

import imgui.ImGui;
import std_msgs.String_;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.detections.supervisePose.SupervisePoseObject;
import us.ihmc.perception.detections.supervisePose.SupervisePoseAPI;
import us.ihmc.perception.detections.supervisePose.SyncedSupervisePoseParameters;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;

public class RDXSupervisePoseSettings
{
   private final SupervisePoseObject object;
   private final ROS2Node ros2Node;

   private final ROS2Publisher<String_> resetRequestPublisher;
   private final SyncedSupervisePoseParameters parameters;

   private final ImGuiUniqueLabelMap labels;
   private final ImBooleanWrapper enabled;
   private final ImBooleanWrapper autoResetEnabled;
   private final ImDoubleWrapper resetDistance;

   public RDXSupervisePoseSettings(ROS2Node ros2Node,
                                   ROS2PeerClockOffsetEstimator ros2ClockOffsetEstimator,
                                   SupervisePoseObject object)
   {
      this.object = object;
      this.ros2Node = ros2Node;

      resetRequestPublisher = ros2Node.createPublisher(SupervisePoseAPI.RESET_REQUEST);

      CRDTInfo crdtInfo = new CRDTInfo(ROS2ActorDesignation.OPERATOR, ros2ClockOffsetEstimator);
      parameters = new SyncedSupervisePoseParameters(ros2Node, crdtInfo, object);

      labels = new ImGuiUniqueLabelMap(getClass());

      enabled = new ImBooleanWrapper(parameters.getEnabled()::getValue,
                                     parameters.getEnabled()::setValue,
                                     imBoolean -> ImGui.checkbox(labels.getHidden("enabled_" + object.name()), imBoolean));

      autoResetEnabled = new ImBooleanWrapper(parameters.getAutoResetEnabled()::getValue,
                                              parameters.getAutoResetEnabled()::setValue,
                                              imBoolean -> ImGui.checkbox(labels.getHidden("auto_reset_enabled_" + object.name()), imBoolean));

      resetDistance = new ImDoubleWrapper(parameters.getResetDistance()::getValue,
                                          parameters.getResetDistance()::setValue,
                                          imDouble -> ImGuiTools.volatileInputDouble(labels.getHidden("reset_distance_" + object.name()),
                                                                                     imDouble,
                                                                                     0.0,
                                                                                     0.0,
                                                                                     "%05.2f"));
   }

   public SupervisePoseObject getObject()
   {
      return object;
   }

   public SyncedSupervisePoseParameters getParameters()
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

      if (ImGui.tableNextColumn()) // Instance
      {
         ImGui.setNextItemWidth(-1.0f);
         ImGui.text(object.instance);
      }

      if (ImGui.tableNextColumn()) // Reset button
      {
         ImGui.setNextItemWidth(-1.0f);
         if (ImGui.button(labels.get("Reset_" + object.name())))
         {
            String_ resetRequest = new String_();
            resetRequest.setData(object.category + " " + object.instance);
            resetRequestPublisher.publish(resetRequest);
         }
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
      ros2Node.destroyPublisher(resetRequestPublisher);
      parameters.close();
   }
}
