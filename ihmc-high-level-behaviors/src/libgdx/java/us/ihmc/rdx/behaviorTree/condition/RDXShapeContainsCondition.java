package us.ihmc.rdx.behaviorTree.condition;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiColorEditFlags;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeDefinition;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeState;
import us.ihmc.behaviors.behaviorTree.condition.ShapeContainsConditionDefinition;
import us.ihmc.behaviors.behaviorTree.condition.ShapeContainsConditionDefinition.ContainsType;
import us.ihmc.behaviors.behaviorTree.condition.ShapeContainsConditionState;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneState;
import us.ihmc.rdx.behaviorTree.RDXCRDTTools;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImIntegerWrapper;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;

public class RDXShapeContainsCondition
{
   private final RDXConditionNode parent;
   private final ConditionNodeDefinition definition;
   private final ConditionNodeState state;
   private final ShapeContainsConditionState shapeState;
   private final ShapeContainsConditionDefinition shapeDefinition;
   private final RDXSelectablePose3DGizmo poseGizmo;
   private final Color sphereColor = new Color(0.0f, 0.0f, 1.0f, 0.5f);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiReferenceFrameLibraryCombo frameComboBox;
   private final ImGuiReferenceFrameLibraryCombo shapeParentFrameComboBox;
   private final ImDoubleWrapper sphereRadiusWidget;
   private final ImIntegerWrapper minPointsWidget;
   private final ImIntegerWrapper maxPointsWidget;
   private final ImBooleanWrapper checkColorWidget;
   private final float[] averageHSVColor = new float[3];
   private ModelInstance sphereModel;
   private double lastSphereRadius = Double.NaN;

   public RDXShapeContainsCondition(RDXConditionNode parent, BehaviorTreeSceneState scene, RDX3DPanel panel3D)
   {
      this.parent = parent;
      this.definition = parent.getDefinition();
      this.state = parent.getState();

      shapeState = state.getShapeContains();
      shapeDefinition = definition.getShapeContains();

      poseGizmo = new RDXSelectablePose3DGizmo();
      poseGizmo.getPoseGizmo().setResizeAutomatically(false);
      poseGizmo.create(panel3D);

      frameComboBox = new ImGuiReferenceFrameLibraryCombo("Frame",
                                                          scene::getAllFrameNames,
                                                          shapeDefinition::getFrameName,
                                                          shapeDefinition::setFrameName);
      shapeParentFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Shape Parent Frame",
                                                                     scene::getAllFrameNames,
                                                                     shapeDefinition::getShapeParentFrameName,
                                                                     shapeState.getShapeFrame()::changeFrame);
      sphereRadiusWidget = new ImDoubleWrapper(shapeDefinition::getSphereRadius,
                                               shapeDefinition::setSphereRadius,
                                               imDouble -> ImGuiTools.volatileInputDouble(labels.get("Sphere Radius"), imDouble, 0.01, 0.1));
      minPointsWidget = new ImIntegerWrapper(shapeDefinition::getMinPoints,
                                             shapeDefinition::setMinPoints,
                                             imInt -> ImGuiTools.volatileInputInt(labels.get("Min Points"), imInt));
      maxPointsWidget = new ImIntegerWrapper(shapeDefinition::getMaxPoints,
                                             shapeDefinition::setMaxPoints,
                                             imInt -> ImGuiTools.volatileInputInt(labels.get("Max Points"), imInt));
      checkColorWidget = new ImBooleanWrapper(shapeDefinition::getCheckColor,
                                              shapeDefinition::setCheckColor,
                                              imBoolean -> ImGui.checkbox(labels.get("Check Color"), imBoolean));
   }

   public void update()
   {
      if (shapeState.getShapeFrame().isChildOfWorld())
      {
         if (poseGizmo.getPoseGizmo().getGizmoFrame() != shapeState.getShapeFrame().getReferenceFrame())
            poseGizmo.getPoseGizmo().setGizmoFrame(shapeState.getShapeFrame().getReferenceFrame());

         RDXCRDTTools.syncGizmoWithBidirectionalField(poseGizmo.getPoseGizmo(), shapeDefinition.getShapeTransformToParent(), definition);

         double sphereRadius = shapeDefinition.getSphereRadius();
         if (sphereModel == null || sphereRadius != lastSphereRadius)
         {
            sphereModel = RDXModelBuilder.createSphere((float) sphereRadius, sphereColor);
            LibGDXTools.setOpacity(sphereModel, sphereColor.a);
            lastSphereRadius = sphereRadius;
         }

         float gradient = switch (shapeDefinition.getContainsType())
         {
            case CONTAINS_FRAME -> shapeState.getFrameIsContained() ? 1.0f : 0.0f;
            case CONTAINS_POINTS -> Math.min(Math.max(shapeState.getNumberOfPointsContained() / 20000.0f, 0.0f), 1.0f);
         };
         sphereColor.set(gradient, 0.0f, 1.0f - gradient, sphereColor.a);
         if (sphereModel != null)
         {
            LibGDXTools.setDiffuseColor(sphereModel, sphereColor);
            LibGDXTools.toLibGDX(poseGizmo.getPoseGizmo().getGizmoFrame().getTransformToRoot(), sphereModel.transform);
         }

         poseGizmo.getPoseGizmo().setTorusRadius(sphereRadius);
      }
      else
      {
         poseGizmo.setSelected(false);
      }
   }

   public void renderImGuiWidgetsInternal()
   {
      ContainsType currentType = shapeDefinition.getContainsType();
      if (ImGui.radioButton("Contains Frame", ContainsType.CONTAINS_FRAME == currentType))
         shapeDefinition.setContainsType(ContainsType.CONTAINS_FRAME);
      ImGui.sameLine();
      if (ImGui.radioButton("Contains Points", ContainsType.CONTAINS_POINTS == currentType))
         shapeDefinition.setContainsType(ContainsType.CONTAINS_POINTS);

      shapeParentFrameComboBox.render();
      ImGui.checkbox(labels.get("Adjust Shape Pose"), poseGizmo.getSelected());
      sphereRadiusWidget.renderImGuiWidget();

      if (currentType == ContainsType.CONTAINS_FRAME)
      {
         frameComboBox.render();
         ImGui.text("Frame contained: " + shapeState.getFrameIsContained());
      }
      else
      {
         ImGui.text("Points contained: " + shapeState.getNumberOfPointsContained());
         minPointsWidget.renderImGuiWidget();
         maxPointsWidget.renderImGuiWidget();
         checkColorWidget.renderImGuiWidget();
         if (definition.getShapeContains().getCheckColor())
         {
            averageHSVColor[0] = shapeState.getAverageHue() / 360.0f;
            averageHSVColor[1] = shapeState.getAverageSaturation();
            averageHSVColor[2] = shapeState.getAverageValue();
            ImGui.beginDisabled();
            ImGui.setNextItemWidth(180.0f);
            ImGui.colorPicker3(labels.get("Average HSV"),
                               averageHSVColor,
                               ImGuiColorEditFlags.InputHSV | ImGuiColorEditFlags.DisplayHSV | ImGuiColorEditFlags.NoLabel | ImGuiColorEditFlags.NoSidePreview);
            ImGui.endDisabled();
         }
         
      }
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (parent.getSelected() && shapeState.getShapeFrame().isChildOfWorld())
         poseGizmo.calculate3DViewPick(input);
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (parent.getSelected() && shapeState.getShapeFrame().isChildOfWorld())
         poseGizmo.process3DViewInput(input);
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if ((state.getIsNextForExecution() || parent.getSelected() || state.getIsExecuting()) && shapeState.getShapeFrame().isChildOfWorld()
          && sphereModel != null)
      {
         sphereModel.getRenderables(renderables, pool);
         poseGizmo.getVirtualRenderables(renderables, pool);
      }
   }

   public void deselectGizmos()
   {
      poseGizmo.setSelected(false);
   }
}
