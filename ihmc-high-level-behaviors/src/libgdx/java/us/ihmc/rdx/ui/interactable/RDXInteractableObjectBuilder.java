package us.ihmc.rdx.ui.interactable;

import imgui.internal.ImGui;
import perception_msgs.msg.dds.ManuallyPlacedSceneNodeMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.sceneGraph.PredefinedSceneNodeLibrary;
import us.ihmc.perception.sceneGraph.SceneGraphAPI;
import us.ihmc.perception.sceneGraph.arUco.ArUcoDetectableNode;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.*;

public class RDXInteractableObjectBuilder extends ImGuiPanel
{
   private final static String WINDOW_NAME = ImGuiTools.uniqueLabel(RDXInteractableObjectBuilder.class, "Object Panel");
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private PredefinedSceneNodeLibrary predefinedSceneNodeLibrary;
   private RDXInteractableObject selectedObject;
   private String selectedObjectName = "";
   private final SortedMap<String, String> nameModelMap = new TreeMap<>();
   private ROS2PublishSubscribeAPI ros2;
   private final ManuallyPlacedSceneNodeMessage objectMessage = new ManuallyPlacedSceneNodeMessage();

   public RDXInteractableObjectBuilder(RDXBaseUI baseUI)
   {
      super(WINDOW_NAME);
      setRenderMethod(this::renderImGuiWidgets);

      selectedObject = new RDXInteractableObject(baseUI);
      predefinedSceneNodeLibrary = PredefinedSceneNodeLibrary.defaultObjects();
      //TODO change this to detectable when node library is updated
      List<ArUcoDetectableNode> availableObjects = predefinedSceneNodeLibrary.getArUcoDetectableNodes();
      for (var object : availableObjects)
         nameModelMap.put(object.getName(), object.getVisualModelFilePath());
   }

   public RDXInteractableObjectBuilder(RDXBaseUI baseUI, ROS2PublishSubscribeAPI ros2)
   {
      super(WINDOW_NAME);
      setRenderMethod(this::renderImGuiWidgets);
      this.ros2 = ros2;

      selectedObject = new RDXInteractableObject(baseUI);
      predefinedSceneNodeLibrary = PredefinedSceneNodeLibrary.defaultObjects();
      //TODO change this to detectable when node library is updated
      List<ArUcoDetectableNode> availableObjects = predefinedSceneNodeLibrary.getArUcoDetectableNodes();
      for (var object : availableObjects)
         nameModelMap.put(object.getName(), object.getVisualModelFilePath());
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Selected: " + (!isAnyObjectSelected() ? "" : (selectedObjectName)));
      ImGui.text("Select object: ");
      for (Map.Entry<String, String> entryNameModel : nameModelMap.entrySet())
      {
         if (ImGui.button(labels.get(entryNameModel.getKey())))
         {
            if (isAnyObjectSelected())
               selectedObject.clear();
            selectedObject.load(entryNameModel.getValue());
            selectedObjectName = entryNameModel.getKey();
         }
         ImGui.separator();
      }
      if (isAnyObjectSelected() && (ImGui.button(labels.get("SET POSE") + "##object")))
      {
         selectedObject.setPose();
         if (ros2 != null)
            publishUpdate();
      }
      ImGui.sameLine();
      if (isAnyObjectSelected() && imgui.ImGui.button(labels.get("RESET TO POSE") + "##object"))
      {
         selectedObject.resetToPose();
         if (ros2 != null)
            publishUpdate();
      }
      ImGui.sameLine();
      if (isAnyObjectSelected() && (ImGui.button(labels.get("DELETE") + "##object")))
      {
         reset();
         if (ros2 != null)
            publishUpdate();
      }
   }

   public void publishUpdate()
   {
         objectMessage.setName(selectedObjectName);
         MessageTools.toMessage(this.getSelectedObject().getTransformToWorld(), objectMessage.getTransformToWorld());
         ros2.publish(SceneGraphAPI.PLACED_SCENE_NODE, objectMessage);
   }

   public void reset()
   {
      selectedObject.clear();
      selectedObjectName = "";
      selectedObject.resetPose();
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }

   public void loadObject(String objectName)
   {
      selectedObject.load(nameModelMap.get(objectName));
      selectedObjectName = objectName;
   }

   public RDXInteractableObject getSelectedObject()
   {
      return selectedObject;
   }

   public boolean isAnyObjectSelected()
   {
      return selectedObject.getModelInstance() != null;
   }

   public String getSelectedObjectName()
   {
      return selectedObjectName;
   }
}
