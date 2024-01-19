package us.ihmc.rdx.ui.interactable;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImFloat;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RDXInteractableObject implements RenderableProvider
{
   private RDXModelInstance modelInstance;
   private final RigidBodyTransform objectTransform = new RigidBodyTransform();
   private final FramePose3D initialObjectPose = new FramePose3D();
   private final ReferenceFrame objectFrame;
   private ReferenceFrame modelInstanceFrame;
   private final RDXSelectablePose3DGizmo selectablePose3DGizmo = new RDXSelectablePose3DGizmo(ReferenceFrame.getWorldFrame(), objectTransform);
   private static final float DEFAULT_DIMENSION = 0.1F;
   protected static final ColorDefinition GHOST_COLOR = ColorDefinitions.parse("0x4B61D1").derive(0.0, 1.0, 1.0, 0.6);
   private PrimitiveRigidBodyShape shape;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImFloat xLength = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat yLength = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat zLength = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat xRadius = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat yRadius = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat zRadius = new ImFloat(DEFAULT_DIMENSION);
   private List<Float> readResizablePrimitiveSize;
   private boolean newScale;

   public RDXInteractableObject(RDXBaseUI baseUI)
   {
      this.objectFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), objectTransform);

      selectablePose3DGizmo.getPoseGizmo().setParentFrame(ReferenceFrame.getWorldFrame());
      selectablePose3DGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
      selectablePose3DGizmo.getPoseGizmo().getTransformToParent().getTranslation().set(objectTransform.getTranslation());
      selectablePose3DGizmo.getPoseGizmo().getTransformToParent().getRotation().set(objectTransform.getRotation());

      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(selectablePose3DGizmo::calculate3DViewPick);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(selectablePose3DGizmo::process3DViewInput);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::update);
      baseUI.getImGuiPanelManager().addPanel(selectablePose3DGizmo.getPoseGizmo().createTunerPanel("object"));
   }

   public void load(String modelFileName)
   {
      ModelData objectModel = RDXModelLoader.loadModelData(modelFileName);
      modelInstance = new RDXModelInstance(new Model(objectModel));
      selectablePose3DGizmo.getSelected().set(true);
   }

   public void load(String modelFileName, RigidBodyTransform visualModelTransform)
   {
      ModelData objectModel = RDXModelLoader.loadModelData(modelFileName);
      modelInstance = new RDXModelInstance(new Model(objectModel));
      modelInstanceFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(objectFrame, visualModelTransform);
      selectablePose3DGizmo.getSelected().set(true);
   }

   public void createVisuals() {
//      , RigidBodyTransform visualModelTransform
      if (modelInstance != null)
         modelInstance.model.dispose();
      switch (shape) {
         case BOX -> modelInstance = new RDXModelInstance(RDXModelBuilder.createBoxOffset(xLength.get(), yLength.get(), zLength.get(), new Point3D(0, 0, zLength.get()/2), Color.WHITE));
         case PRISM -> modelInstance = new RDXModelInstance(RDXModelBuilder.createPrism(xLength.get(), yLength.get(), zLength.get(), new Point3D(0, 0, -zLength.get() / 2), Color.WHITE));
         case CYLINDER -> modelInstance = new RDXModelInstance(RDXModelBuilder.createCylinder(zLength.get(), xRadius.get(), new Point3D(0, 0, -zLength.get() / 2), Color.WHITE));
         case CONE -> modelInstance = new RDXModelInstance(RDXModelBuilder.createCone(zLength.get(), xRadius.get(), new Point3D(0, 0, -zLength.get() / 2), Color.WHITE));
         case ELLIPSOID -> modelInstance = new RDXModelInstance(RDXModelBuilder.createEllipsoid(xRadius.get(), yRadius.get(), zRadius.get(), new Point3D(), Color.WHITE));
      }
      modelInstance.setColor(GHOST_COLOR);
      modelInstanceFrame = objectFrame;
//      ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(objectFrame, visualModelTransform);
      selectablePose3DGizmo.getSelected().set(true);
   }

   public void updateVisuals() {
      ImGui.text("Modify shape:");
      switch (shape)
      {
         case BOX, PRISM ->
         {
            if (ImGuiTools.volatileInputFloat(labels.get("depth"), xLength) ||
                ImGuiTools.volatileInputFloat(labels.get("width"), yLength) ||
                ImGuiTools.volatileInputFloat(labels.get("height"), zLength)) {
               createVisuals();
               newScale = true;
            }
         }
         case CYLINDER, CONE ->
         {
            if (ImGuiTools.volatileInputFloat(labels.get("radius"), xRadius) ||
                ImGuiTools.volatileInputFloat(labels.get("height"), zLength)) {
               createVisuals();
               newScale = true;
            }
         }
         case ELLIPSOID ->
         {
            if (ImGuiTools.volatileInputFloat(labels.get("xRadius"), xRadius) ||
                ImGuiTools.volatileInputFloat(labels.get("yRadius"), yRadius) ||
                ImGuiTools.volatileInputFloat(labels.get("zRadius"), zRadius)) {
               createVisuals();
               newScale = true;
            }
         }
      }
   }

   public boolean isNewScale()
   {
      return newScale;
   }

   public void setNewScale(boolean newScale)
   {
      this.newScale = newScale;
   }

   public PrimitiveRigidBodyShape getShape()
   {
      return shape;
   }

   public void setShape(PrimitiveRigidBodyShape newShape)
   {
      this.shape = newShape;
   }

   public ArrayList<Float> getResizablePrimitiveSize()
   {
      return new ArrayList<>(Arrays.asList(xLength.get(), yLength.get(), zLength.get(), xRadius.get(), yRadius.get(), zRadius.get()));
   }

   public void setReadResizablePrimitiveSize(List<Float> newSize)
   {
      this.readResizablePrimitiveSize = newSize;
   }

   public List<Float> getReadResizablePrimitiveSize()
   {
      return readResizablePrimitiveSize;
   }

   public void clear()
   {
      shape = null;
      modelInstance.model.dispose();
      modelInstance = null;
      selectablePose3DGizmo.getSelected().set(false);
   }

   public void update(ImGui3DViewInput input)
   {
      selectablePose3DGizmo.getPoseGizmo().update();
      objectTransform.set(selectablePose3DGizmo.getPoseGizmo().getTransformToParent());
      objectFrame.update();
      if (modelInstance != null)
         LibGDXTools.toLibGDX(modelInstanceFrame.getTransformToRoot(), modelInstance.transform);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modelInstance != null)
         modelInstance.getRenderables(renderables, pool);

      if (selectablePose3DGizmo.getSelected().get())
         selectablePose3DGizmo.getVirtualRenderables(renderables, pool);
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return objectTransform;
   }

   public FramePose3D getInitialPose()
   {
      return initialObjectPose;
   }

   public RigidBodyTransform getInitialTransformToWorld()
   {
      RigidBodyTransform initialObjectTransformToWorld = new RigidBodyTransform();
      initialObjectPose.get(initialObjectTransformToWorld);
      return initialObjectTransformToWorld;
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public void setInitialPose()
   {
      initialObjectPose.set(selectablePose3DGizmo.getPoseGizmo().getTransformToParent());
   }

   public void setPose(RigidBodyTransform transformToWorld)
   {
      initialObjectPose.set(transformToWorld);
      selectablePose3DGizmo.getPoseGizmo().getTransformToParent().set(initialObjectPose);
   }

   public void resetToInitialPose()
   {
      selectablePose3DGizmo.getPoseGizmo().getTransformToParent().set(initialObjectPose);
   }

   public void resetPose()
   {
      selectablePose3DGizmo.getPoseGizmo().getTransformToParent().setToZero();
      initialObjectPose.setToZero();
   }

   public void switchGizmoVisualization()
   {
      selectablePose3DGizmo.getSelected().set(!selectablePose3DGizmo.getSelected().get());
   }
}