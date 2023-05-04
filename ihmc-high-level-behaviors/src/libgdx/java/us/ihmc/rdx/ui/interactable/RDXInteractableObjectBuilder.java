package us.ihmc.rdx.ui.interactable;

import imgui.internal.ImGui;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.PredefinedSceneNodeLibrary;
import us.ihmc.perception.sceneGraph.arUco.ArUcoDetectableNode;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RDXInteractableObjectBuilder extends ImGuiPanel
{
   private final static String WINDOW_NAME = ImGuiTools.uniqueLabel(RDXInteractableObjectBuilder.class, "Objects");
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private PredefinedSceneNodeLibrary predefinedSceneNodeLibrary;
   private RDXInteractableObject selectedObject;
   private final String selectedObjectName = new String();
   private final HashMap<String, String> nameModelMap = new HashMap<>();

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
      ImGui.text("Selected: " + (selectedObject == null ? "" : (selectedObjectName)));

      for (Map.Entry<String, String> entryNameModel : nameModelMap.entrySet())
      {
         if (ImGui.button(labels.get(entryNameModel.getKey())))
         {
            if (selectedObject.getModelInstance() != null)
               selectedObject.clear();
            selectedObject.load(entryNameModel.getValue());
         }
         ImGui.separator();
      }
      if (selectedObject != null && (ImGui.button("Delete object") || ImGui.isKeyReleased(ImGuiTools.getDeleteKey())))
      {
         selectedObject.clear();
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
}
