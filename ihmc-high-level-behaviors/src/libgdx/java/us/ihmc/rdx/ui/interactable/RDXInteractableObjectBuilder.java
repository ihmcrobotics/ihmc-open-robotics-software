package us.ihmc.rdx.ui.interactable;

import imgui.internal.ImGui;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.PredefinedSceneNodeLibrary;
import us.ihmc.perception.sceneGraph.arUco.ArUcoDetectableNode;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.*;

public class RDXInteractableObjectBuilder extends RDXPanel
{
   private final static String WINDOW_NAME = "Object Panel";
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final PredefinedSceneNodeLibrary predefinedSceneNodeLibrary;
   private RDXInteractableObject selectedObject;
   private final SortedMap<String, String> nameModelMap = new TreeMap<>();
   private final SortedMap<String, RigidBodyTransform> visualModelTransformMap = new TreeMap<>();
   private String selectedObjectName = "";
   private final TypedNotification<String> selectedObjectChanged = new TypedNotification<>();

   public RDXInteractableObjectBuilder(RDXBaseUI baseUI, PredefinedSceneNodeLibrary predefinedSceneNodeLibrary)
   {
      super(WINDOW_NAME);
      setRenderMethod(this::renderImGuiWidgets);
      this.predefinedSceneNodeLibrary = predefinedSceneNodeLibrary;

      selectedObject = new RDXInteractableObject(baseUI);
      //TODO change this to detectable when node library is updated
      List<ArUcoDetectableNode> availableObjects = predefinedSceneNodeLibrary.getArUcoDetectableNodes();
      for (var object : availableObjects)
      {
         nameModelMap.put(object.getName(), object.getVisualModelFilePath());
         visualModelTransformMap.put(object.getName(), object.getVisualModelToNodeFrameTransform());
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Selected: " + (!isAnyObjectSelected() ? "" : (selectedObjectChanged.read())));
      ImGui.text("Select object: ");
      for (Map.Entry<String, String> entryNameModel : nameModelMap.entrySet())
      {
         if (ImGui.button(labels.get(entryNameModel.getKey())))
         {
            if (isAnyObjectSelected())
               selectedObject.clear();
            selectedObject.load(entryNameModel.getValue(), visualModelTransformMap.get(entryNameModel.getKey()));
            // Notify that the object selection has been updated
            selectedObjectName = entryNameModel.getKey();
            selectedObjectChanged.set(selectedObjectName);

            // check if object is currently detected, if so use the detected pose as initial pose
            List<DetectableSceneNode> detectableSceneObjects = predefinedSceneNodeLibrary.getDetectableSceneNodes();
            for (DetectableSceneNode detectableSceneObject : detectableSceneObjects)
            {
               if (detectableSceneObject.getName().equals(selectedObjectChanged.read()) && detectableSceneObject.getCurrentlyDetected())
               {
                  LogTools.info(detectableSceneObject.getName());
                  FramePose3D sceneObjectPose = new FramePose3D(detectableSceneObject.getNodeFrame());
                  sceneObjectPose.changeFrame(ReferenceFrame.getWorldFrame());
                  RigidBodyTransform sceneObjectToWorldTransform = new RigidBodyTransform();
                  sceneObjectPose.get(sceneObjectToWorldTransform);
                  this.getSelectedObject().setPose(sceneObjectToWorldTransform);
               }
            }
         }
         ImGui.separator();
      }
      if (isAnyObjectSelected() && (ImGui.button(labels.get("Set Initial Pose") + "##object")))
      {
         selectedObject.setInitialPose();
      }
      ImGui.sameLine();
      if (isAnyObjectSelected() && (ImGui.button(labels.get("Reset To Initial Pose") + "##object")))
      {
         selectedObject.resetToInitialPose();
      }
      ImGui.sameLine();
      if (isAnyObjectSelected() && (ImGui.button(labels.get("Delete") + "##object")))
      {
         reset();
      }
      if (isAnyObjectSelected() && (ImGui.button(labels.get("Hide/Show Gizmo") + "##object")))
      {
         selectedObject.switchGizmoVisualization();
      }
   }

   public void reset()
   {
      selectedObject.clear();
      selectedObjectName = "";
      selectedObjectChanged.set(selectedObjectName);
      selectedObject.resetPose();
   }

   public void resetPose()
   {
      selectedObject.resetPose();
   }

   public TypedNotification<String> getSelectedObjectNotification()
   {
      return selectedObjectChanged;
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }

   public void loadObject(String objectName)
   {
      selectedObject.load(nameModelMap.get(objectName), visualModelTransformMap.get(objectName));
      selectedObjectName = objectName;
      selectedObjectChanged.set(selectedObjectName);
   }

   public RDXInteractableObject getSelectedObject()
   {
      return selectedObject;
   }

   public String getSelectedObjectName()
   {
      return selectedObjectName;
   }

   public boolean isAnyObjectSelected()
   {
      return selectedObject.getModelInstance() != null;
   }
}