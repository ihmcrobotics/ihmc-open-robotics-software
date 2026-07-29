package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.BehaviorTreeSceneObjectStateMessage;
import behavior_msgs.PersistentDetectionStatusMessage;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectState;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectType;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.rdx.RDX3DSituatedText;
import us.ihmc.rdx.behaviorTree.RDXCRDTTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;

import java.util.Random;

public class RDXBehaviorTreeSceneObject extends BehaviorTreeSceneObjectState
{
   private static final Random random = new Random();
   private final RDXBaseUI baseUI;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(RDXBehaviorTreeSceneObject.class);
   private final RDXSelectablePose3DGizmo gizmo;
   protected Model model;
   protected RDXModelInstance modelInstance;
   private final ModelInstance frameGraphic;
   private final RDX3DSituatedText textLabel = new RDX3DSituatedText();

   private final PersistentDetectionStatusMessage persistentDetection = new PersistentDetectionStatusMessage();

   public RDXBehaviorTreeSceneObject(long id, CRDTInfo crdtInfo, BehaviorTreeSceneObjectDefinitionMessage definition, RDXBaseUI baseUI)
   {
      super(id, crdtInfo, definition);

      this.baseUI = baseUI;

      gizmo = new RDXSelectablePose3DGizmo();
      gizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
      gizmo.getPoseGizmo().setGizmoFrame(referenceFrame);

      frameGraphic = RDXModelBuilder.createCoordinateFrameInstance(0.2, LibGDXTools.toLibGDX(YoAppearance.randomColor(random)));
      if (getObjectType() == BehaviorTreeSceneObjectType.FOUNDATION_POSE)
      {
         String modelName = getFoundationPoseObjectType().meshDirectory;
         String modelPath = "environmentObjects/" + modelName + "/" + modelName + ".glb";
         model = RDXModelLoader.load(modelPath);
         modelInstance = new RDXModelInstance(model);
      }
   }

   public void update()
   {
      RDXCRDTTools.syncGizmoWithBidirectionalField(gizmo.getPoseGizmo(), transform, this);
      if (modelInstance != null)
         modelInstance.setTransformToWorldFrame(transform.getValueUnsafe());

      LibGDXTools.toLibGDX(transform.getValueUnsafe(), frameGraphic.transform);

      textLabel.setTextWithoutCache(getName());
      textLabel.setPositionFacingCamera(baseUI.getPrimary3DPanel().getCamera3D(),
                                        transform.getValueReadOnly().getTranslation().getX(),
                                        transform.getValueReadOnly().getTranslation().getY(),
                                        transform.getValueReadOnly().getTranslation().getZ());
   }

   public boolean renderImGuiWidgets()
   {
      boolean remove = false;

      ImGui.text("%s %d%s%s".formatted(getName(), getID(), isValid() ? "" : " (INVALID)", isFrozen() ? " (FROZEN)" : ""));
      ImGui.sameLine();
      if (ImGuiTools.smallCheckbox(labels.getHidden("Select%s%d".formatted(getName(), getID())), getGizmo().isSelected()))
         getGizmo().setSelected(!getGizmo().isSelected());
      ImGui.sameLine();
      ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.DARK_RED);
      if (ImGuiTools.textWithUnderlineOnHover("X") && ImGui.isMouseClicked(ImGuiMouseButton.Left))
         remove = true;
      ImGui.popStyleColor();
      ImGui.indent();
      ImGui.text("World frame: (%.2f, %.2f, %.2f) YPR: (%.2f, %.2f, %.2f)".formatted(
            getTransformToWorld().getTranslationX(), getTransformToWorld().getTranslation().getY(), getTransformToWorld().getTranslation().getZ(),
            getTransformToWorld().getRotation().getYaw(), getTransformToWorld().getRotation().getPitch(), getTransformToWorld().getRotation().getRoll()
      ));
      renderDetectionInfo();
      ImGui.unindent();

      return remove;
   }

   protected void renderDetectionInfo()
   {
      RDXBehaviorTreeScene.renderPersistentDetection(getPersistentDetection());
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modelInstance != null)
         modelInstance.getRenderables(renderables, pool);
      frameGraphic.getRenderables(renderables, pool);
      textLabel.getRenderables(renderables, pool);
   }

   public void destroy()
   {
      gizmo.setSelected(false);
      gizmo.destroyDefault(baseUI.getPrimary3DPanel());
      if (model != null)
         model.dispose();
   }

   @Override
   public void fromMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.fromMessage(message);

      persistentDetection.set(message.getPersistentDetection());
   }

   public RDXSelectablePose3DGizmo getGizmo()
   {
      return gizmo;
   }

   public PersistentDetectionStatusMessage getPersistentDetection()
   {
      return persistentDetection;
   }
}
