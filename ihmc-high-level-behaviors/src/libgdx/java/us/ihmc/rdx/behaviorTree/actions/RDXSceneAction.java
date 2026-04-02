package us.ihmc.rdx.behaviorTree.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.behaviors.behaviorTree.action.actions.SceneActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.SceneActionDefinition.SceneActionType;
import us.ihmc.behaviors.behaviorTree.action.actions.SceneActionState;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectDefinition.CompositeFrameType;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectDefinition;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectType;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.perception.detections.yolo.SyncedYOLOv8ModelParameters;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.behaviorTree.RDXCRDTTools;
import us.ihmc.rdx.imgui.ImFloatWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImIntegerWrapper;
import us.ihmc.rdx.imgui.ImStringWrapper;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.widgets.ImGuiSceneActionWidget;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;

public class RDXSceneAction extends RDXActionNode<SceneActionState, SceneActionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiSceneActionWidget widget = new ImGuiSceneActionWidget();
   private final ImInt imObjectType = new ImInt(0);
   private final ImInt imYOLOModel = new ImInt(0);
   private final ImInt imYOLOClass = new ImInt(0);
   private final ImInt imFPType = new ImInt(0);
   private final ImInt imCompositeFrameType = new ImInt(0);
   private final String[] sceneObjectTypeNames;
   private final String[] fpTypeNames;
   private final String[] compositeFrameTypeNames;
   private final String[] availableYOLOModelNames;
   private final String[][] availableYOLOClasses;
   private final ImFloatWrapper timeoutWidget;
   private final ImIntegerWrapper minHistorySizeWidget;
   private final ImIntegerWrapper minPostPointsWidget;
   private final ImIntegerWrapper minRecessPointsWidget;
   private final ImStringWrapper compositeFrameNameWidget;
   private final ImGuiReferenceFrameLibraryCombo compositeFrameAComboBox;
   private final ImGuiReferenceFrameLibraryCombo compositeFrameBComboBox;
   private final ImFloatWrapper compositeFrameDistanceWidget;
   private final RDXSelectablePose3DGizmo nominalObjectPoseGizmo;

   public RDXSceneAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new SceneActionState(id, rootNode.getState()), rootNode);

      nominalObjectPoseGizmo = new RDXSelectablePose3DGizmo(definition.getNominalObjectPose().getValueUnsafe(), scene.findFrameByName("Walking"));
      nominalObjectPoseGizmo.create(panel3D);

      BehaviorTreeSceneObjectType[] sceneObjectTypes = BehaviorTreeSceneObjectType.values;
      sceneObjectTypeNames = new String[sceneObjectTypes.length];
      for (int i = 0; i < sceneObjectTypes.length; i++)
         sceneObjectTypeNames[i] = sceneObjectTypes[i].name();

      IsaacROSFoundationPoseObject[] values = IsaacROSFoundationPoseObject.values();
      fpTypeNames = new String[values.length];
      for (int i = 0; i < values.length; i++)
         fpTypeNames[i] = values[i].titleCaseName;
      compositeFrameTypeNames = new String[CompositeFrameType.values.length];
      for (int i = 0; i < CompositeFrameType.values.length; i++)
         compositeFrameTypeNames[i] = CompositeFrameType.values[i].name();

      SyncedYOLOv8ModelParameters[] syncableParameters = definition.getSyncableYOLOModelParameters();
      availableYOLOModelNames = new String[syncableParameters.length];
      availableYOLOClasses = new String[availableYOLOModelNames.length][];
      for (int i = 0; i < syncableParameters.length; i++)
      {
         availableYOLOModelNames[i] = syncableParameters[i].getModelName();
         String[] detectableObjectClasses = syncableParameters[i].getDetectableObjectClasses();
         availableYOLOClasses[i] = new String[detectableObjectClasses.length];
         for (int j = 0; j < detectableObjectClasses.length; j++)
            availableYOLOClasses[i][j] = detectableObjectClasses[j];
      }

      timeoutWidget = new ImFloatWrapper(definition::getTimeout,
                                         definition::setTimeout,
                                         imFloat -> ImGui.inputFloat(labels.get("Timeout"), imFloat));
      minHistorySizeWidget = new ImIntegerWrapper(definition::getMinimumHistorySize,
                                                  definition::setMinimumHistorySize,
                                                  imInteger -> ImGui.inputInt(labels.get("Minimum History Size"), imInteger));
      minPostPointsWidget = new ImIntegerWrapper(() -> definition.getSceneObjectDefinition().getMinPostPoints(),
                                                 value -> definition.getSceneObjectDefinition().setMinPostPoints(value),
                                                 imInteger -> ImGui.inputInt(labels.get("Min Post Points"), imInteger));
      minRecessPointsWidget = new ImIntegerWrapper(() -> definition.getSceneObjectDefinition().getMinRecessPoints(),
                                                   value -> definition.getSceneObjectDefinition().setMinRecessPoints(value),
                                                   imInteger -> ImGui.inputInt(labels.get("Min Recess Points"), imInteger));
      compositeFrameNameWidget = new ImStringWrapper(() -> definition.getSceneObjectDefinition().getCompositeFrameName(),
                                                     value -> definition.getSceneObjectDefinition().setCompositeFrameName(value),
                                                     imString -> ImGui.inputText(labels.get("Composite Frame Name"), imString));
      compositeFrameAComboBox = new ImGuiReferenceFrameLibraryCombo("Frame A",
                                                                    scene::getAllFrameNames,
                                                                    () -> definition.getSceneObjectDefinition().getCompositeFrameA(),
                                                                    value -> definition.getSceneObjectDefinition().setCompositeFrameA(value));
      compositeFrameBComboBox = new ImGuiReferenceFrameLibraryCombo("Frame B",
                                                                    scene::getAllFrameNames,
                                                                    () -> definition.getSceneObjectDefinition().getCompositeFrameB(),
                                                                    value -> definition.getSceneObjectDefinition().setCompositeFrameB(value));
      compositeFrameDistanceWidget = new ImFloatWrapper(() -> definition.getSceneObjectDefinition().getCompositeDistance(),
                                                        value -> definition.getSceneObjectDefinition().setCompositeDistance(value),
                                                        imFloat -> ImGui.inputFloat(labels.get("Distance"), imFloat));
   }

   @Override
   public void update()
   {
      super.update();

      nominalObjectPoseGizmo.getPoseGizmo().setParentFrame(scene.findFrameByName("Walking"));

      RDXCRDTTools.syncGizmoWithBidirectionalField(nominalObjectPoseGizmo.getPoseGizmo(), definition.getNominalObjectPose(), definition);
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();
      super.renderEditableName();
      ImGui.sameLine();
      widget.render();
      renderRowEnd();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      SceneActionType currentActionType = definition.getSceneActionType().getValue();
      if (ImGui.beginCombo(labels.get("Action Type"), currentActionType.name()))
      {
         for (SceneActionType value : SceneActionType.values)
         {
            if (ImGui.selectable(value.name(), value == currentActionType))
               definition.getSceneActionType().setValue(value);
         }
         ImGui.endCombo();
      }

      if (definition.getSceneActionType().getValue() != SceneActionType.CLEAR_SCENE
       && definition.getSceneActionType().getValue() != SceneActionType.FREEZE_SCENE)
      {
         BehaviorTreeSceneObjectDefinition objectDefinition = definition.getSceneObjectDefinition();

         ImGui.pushItemWidth(200.0f);
         imObjectType.set(objectDefinition.getObjectType().ordinal());
         if (ImGui.combo(labels.get("Setup Object Type"), imObjectType, sceneObjectTypeNames))
            objectDefinition.setObjectType(BehaviorTreeSceneObjectType.values[imObjectType.get()]);
         ImGui.popItemWidth();

         ImGui.pushItemWidth(200.0f);
         imYOLOModel.set(-1);
         for (int i = 0; i < availableYOLOModelNames.length; i++)
            if (availableYOLOModelNames[i].equals(objectDefinition.getYoloModelName()))
               imYOLOModel.set(i);
         if (ImGui.combo(labels.get("YOLO Model"), imYOLOModel, availableYOLOModelNames))
            objectDefinition.setYoloModelName(availableYOLOModelNames[imYOLOModel.get()]);
         ImGui.popItemWidth();

         if (objectDefinition.getObjectType() == BehaviorTreeSceneObjectType.YOLO_ONLY)
         {
            ImGui.pushItemWidth(200.0f);
            imYOLOClass.set(-1);
            for (int i = 0; i < availableYOLOClasses[imYOLOModel.get()].length; i++)
               if (availableYOLOClasses[imYOLOModel.get()][i].equals(objectDefinition.getYoloClassName()))
                  imYOLOClass.set(i);
            if (ImGui.combo(labels.get("YOLO Class"), imYOLOClass, availableYOLOClasses[imYOLOModel.get()]))
               objectDefinition.setYoloClassName(availableYOLOClasses[imYOLOModel.get()][imYOLOClass.get()]);
            ImGui.popItemWidth();
         }
         else if (objectDefinition.getObjectType() == BehaviorTreeSceneObjectType.FOUNDATION_POSE)
         {
            ImGui.pushItemWidth(200.0f);
            imFPType.set(objectDefinition.getFoundationPoseObjectType().ordinal());
            if (ImGui.combo(labels.get("FoundationPose Type"), imFPType, fpTypeNames))
               objectDefinition.setFoundationPoseObjectType(IsaacROSFoundationPoseObject.values()[imFPType.get()]);
            ImGui.popItemWidth();
         }
         else if (objectDefinition.getObjectType() == BehaviorTreeSceneObjectType.DOOR_FRAME)
         {
            ImGui.pushItemWidth(100.0f);
            minPostPointsWidget.renderImGuiWidget();
            minRecessPointsWidget.renderImGuiWidget();
            ImGui.popItemWidth();
         }
         else if (objectDefinition.getObjectType() == BehaviorTreeSceneObjectType.COMPOSITE_FRAME)
         {
            ImGui.pushItemWidth(200.0f);
            imCompositeFrameType.set(objectDefinition.getCompositeFrameType().ordinal());
            if (ImGui.combo(labels.get("Composite Frame Type"), imCompositeFrameType, compositeFrameTypeNames))
               objectDefinition.setCompositeFrameType(CompositeFrameType.values[imCompositeFrameType.get()]);
            compositeFrameNameWidget.renderImGuiWidget();
            ImGui.popItemWidth();
            if (objectDefinition.getCompositeFrameType() == CompositeFrameType.APPROACH)
               ImGui.text("From:");
            if (objectDefinition.getCompositeFrameType() == CompositeFrameType.HYBRID)
               ImGui.text("Position:");
            compositeFrameAComboBox.render();
            if (objectDefinition.getCompositeFrameType() == CompositeFrameType.APPROACH)
               ImGui.text("To:");
            if (objectDefinition.getCompositeFrameType() == CompositeFrameType.HYBRID)
               ImGui.text("Orientation:");
            compositeFrameBComboBox.render();

            ImGui.pushItemWidth(100.0f);
            if (objectDefinition.getCompositeFrameType() == CompositeFrameType.APPROACH)
               compositeFrameDistanceWidget.renderImGuiWidget();
            ImGui.popItemWidth();
         }

         ImGui.pushItemWidth(100.0f);
         timeoutWidget.renderImGuiWidget();
         minHistorySizeWidget.renderImGuiWidget();
         ImGui.checkbox(labels.get("Adjust Nominal Object Pose"), nominalObjectPoseGizmo.getSelected());
         ImGui.popItemWidth();
      }
   }

   @Override
   public void deselectGizmos()
   {
      nominalObjectPoseGizmo.setSelected(false);
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (getSelected())
         nominalObjectPoseGizmo.calculate3DViewPick(input);
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (getSelected())
         nominalObjectPoseGizmo.process3DViewInput(input);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (getSelected())
      {
         nominalObjectPoseGizmo.getVirtualRenderables(renderables, pool);
      }
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "Scene Action";
   }
}
