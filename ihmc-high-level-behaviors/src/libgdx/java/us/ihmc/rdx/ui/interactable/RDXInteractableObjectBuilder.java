package us.ihmc.rdx.ui.interactable;

import imgui.internal.ImGui;
import us.ihmc.perception.sceneGraph.PredefinedSceneNodeLibrary;
import us.ihmc.perception.sceneGraph.arUco.ArUcoDetectableNode;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.*;

public class RDXInteractableObjectBuilder extends ImGuiPanel
{
   private final static String WINDOW_NAME = ImGuiTools.uniqueLabel(RDXInteractableObjectBuilder.class, "Objects");
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private PredefinedSceneNodeLibrary predefinedSceneNodeLibrary;
   private RDXInteractableObject selectedObject;
   private String selectedObjectName;
   private final SortedMap<String, String> nameModelMap = new TreeMap<>();

   public RDXInteractableObjectBuilder(RDXBaseUI baseUI)
   {
      super(WINDOW_NAME);
      setRenderMethod(this::renderImGuiWidgets);

      selectedObject = new RDXInteractableObject(baseUI);
      predefinedSceneNodeLibrary = PredefinedSceneNodeLibrary.defaultObjects();
      //TODO change this to detectable when library is updated
      List<ArUcoDetectableNode> availableObjects = predefinedSceneNodeLibrary.getArUcoDetectableNodes();
      for (var object : availableObjects)
         nameModelMap.put(object.getName(), object.getVisualModelFilePath());
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Selected: " + (!isAnyObjectSelected() ? "" : (selectedObjectName)));

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
      if (isAnyObjectSelected() && (ImGui.button("Delete object")))
      {
         selectedObject.clear();
         selectedObjectName = "";
      }
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

}
