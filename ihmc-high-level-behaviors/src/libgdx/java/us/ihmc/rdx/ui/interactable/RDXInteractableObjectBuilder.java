package us.ihmc.rdx.ui.interactable;

import imgui.internal.ImGui;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.perception.sceneGraph.PredefinedSceneNodeLibrary;
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
      if (isAnyObjectSelected() && (ImGui.button(labels.get("SET INITIAL POSE") + "##object")))
      {
         selectedObject.setInitialPose();
      }
      if (isAnyObjectSelected() && (ImGui.button(labels.get("DELETE") + "##object")))
      {
         reset();
      }
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
