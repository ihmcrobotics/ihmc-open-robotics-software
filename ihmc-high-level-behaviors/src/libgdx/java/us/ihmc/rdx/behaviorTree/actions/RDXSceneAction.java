package us.ihmc.rdx.behaviorTree.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImInt;
import org.yaml.snakeyaml.Yaml;
import us.ihmc.behaviors.behaviorTree.action.actions.SceneActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.SceneActionDefinition.SceneActionType;
import us.ihmc.behaviors.behaviorTree.action.actions.SceneActionState;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectDefinition;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectType;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.perception.detections.yolo.YOLOv8Tools;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.behaviorTree.RDXCRDTTools;
import us.ihmc.rdx.imgui.ImFloatWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImIntegerWrapper;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.widgets.ImGuiSceneActionWidget;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.util.List;
import java.util.Map;
import java.util.regex.Pattern;

public class RDXSceneAction extends RDXActionNode<SceneActionState, SceneActionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiSceneActionWidget widget = new ImGuiSceneActionWidget();
   private final ImInt imYOLOModel = new ImInt(0);
   private final ImInt imYOLOClass = new ImInt(0);
   private final ImInt imFPType = new ImInt(0);
   private final String[] fpTypeNames;
   private final String[] availableYOLOModelNames;
   private final String[][] availableYOLOClasses;
   private final ImFloatWrapper timeoutWidget;
   private final ImIntegerWrapper minHistorySizeWidget;
   private final RDXSelectablePose3DGizmo nominalObjectPoseGizmo;

   public RDXSceneAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new SceneActionState(id, rootNode.getState()), rootNode);

      nominalObjectPoseGizmo = new RDXSelectablePose3DGizmo(definition.getNominalObjectPose().getValueUnsafe(), scene.findFrameByName("Walking"));
      nominalObjectPoseGizmo.create(panel3D);

      IsaacROSFoundationPoseObject[] values = IsaacROSFoundationPoseObject.values();
      fpTypeNames = new String[values.length];
      for (int i = 0; i < values.length; i++)
         fpTypeNames[i] = values[i].titleCaseName;

      List<URL> yoloModelDirectories = YOLOv8Tools.getYOLOModelDirectories();
      availableYOLOModelNames = new String[yoloModelDirectories.size()];
      for (int i = 0; i < yoloModelDirectories.size(); i++)
      {
         String[] path = yoloModelDirectories.get(i).getPath().split(Pattern.quote(File.separator));
         availableYOLOModelNames[i] = path[path.length - 1];
      }

      availableYOLOClasses = new String[availableYOLOModelNames.length][];
      for (int i = 0; i < availableYOLOModelNames.length; i++)
      {
         try (InputStream classNamesFile = YOLOv8Tools.getClassNamesFile(yoloModelDirectories.get(i)).openStream())
         {
            Yaml yaml = new Yaml();
            Map<String, List<Object>> classNamesData = yaml.load(classNamesFile);
            List<Object> names = classNamesData.get("names");
            availableYOLOClasses[i] = new String[names.size()];
            for (int j = 0; j < names.size(); j++)
               availableYOLOClasses[i][j] = names.get(j).toString();
         }
         catch (IOException e)
         {
            DefaultExceptionHandler.MESSAGE_AND_STACKTRACE.handleException(e);
         }
      }

      timeoutWidget = new ImFloatWrapper(definition::getTimeout,
                                         definition::setTimeout,
                                         imFloat -> ImGui.inputFloat(labels.get("Timeout"), imFloat));
      minHistorySizeWidget = new ImIntegerWrapper(definition::getMinimumHistorySize,
                                                  definition::setMinimumHistorySize,
                                                  imInteger -> ImGui.inputInt(labels.get("Minimum History Size"), imInteger));
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

      if (definition.getSceneActionType().getValue() != SceneActionType.CLEAR_SCENE)
      {
         BehaviorTreeSceneObjectDefinition objectDefinition = definition.getSceneObjectDefinition();

         ImGui.text("Setup Object Type:");
         for (BehaviorTreeSceneObjectType type : BehaviorTreeSceneObjectType.values)
            if (ImGui.radioButton(type.name(), objectDefinition.getObjectType() == type))
               objectDefinition.setObjectType(type);

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
