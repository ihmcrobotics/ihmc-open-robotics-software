package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.input.ImGui3DViewPickResult;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXModelInstance;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class GDXInteractableFrameModel
{
   private ReferenceFrame representativeReferenceFrame;
   private final FramePose3D tempFramePose = new FramePose3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private RigidBodyTransform transformToParent;
   private GDXModelInstance modelInstance;
   private GDXInteractableHighlightModel highlightModelInstance;
   private GDXMousePickRayCollisionCalculator collisionCalculator;
   private boolean isMouseHovering;
   private GDXSelectablePose3DGizmo selectablePose3DGizmo;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();

   public void create(ReferenceFrame parentFrame,
                      GDX3DPanel panel3D,
                      GDXModelInstance modelInstance,
                      GDXMousePickRayCollisionCalculator collisionCalculator)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      ReferenceFrame referenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(parentFrame, transform);
      create(referenceFrame, transform, panel3D, modelInstance, collisionCalculator);
   }

   public void create(ReferenceFrame referenceFrameToRepresent,
                      RigidBodyTransform transformToParentToModify,
                      GDX3DPanel panel3D,
                      GDXModelInstance modelInstance,
                      GDXMousePickRayCollisionCalculator collisionCalculator)
   {
      representativeReferenceFrame = referenceFrameToRepresent;
      transformToParent = transformToParentToModify;
      this.modelInstance = modelInstance;
      this.collisionCalculator = collisionCalculator;

      highlightModelInstance = new GDXInteractableHighlightModel(modelInstance.model);
      selectablePose3DGizmo = new GDXSelectablePose3DGizmo(representativeReferenceFrame, transformToParent);
      selectablePose3DGizmo.create(panel3D);
      panel3D.addImGui3DViewPickCalculator(this::calculate3DViewPick);
      panel3D.addImGui3DViewInputProcessor(this::process3DViewInput);
      panel3D.getScene().addRenderableProvider(this::getModelRenderables, GDXSceneLevel.MODEL);
      panel3D.getScene().addRenderableProvider(this::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
   }

   private void calculate3DViewPick(ImGui3DViewInput input)
   {
      selectablePose3DGizmo.calculate3DViewPick(input);

      Line3DReadOnly pickRay = input.getPickRayInWorld();
      double closestCollisionDistance = collisionCalculator.calculateClosestCollition(pickRay);
      if (!Double.isNaN(closestCollisionDistance))
      {
         pickResult.setDistanceToCamera(closestCollisionDistance);
         input.addPickResult(pickResult);
      }
   }

   private void process3DViewInput(ImGui3DViewInput input)
   {
      isMouseHovering = pickResult == input.getClosestPick();

      selectablePose3DGizmo.process3DViewInput(input, isMouseHovering);
      tempFramePose.setToZero(representativeReferenceFrame);
      tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      tempFramePose.get(tempTransform);

      GDXTools.toGDX(tempTransform, modelInstance.transform);
      highlightModelInstance.setPose(tempTransform);
   }

   private void getModelRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      modelInstance.getRenderables(renderables, pool);
   }

   private void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (isMouseHovering || selectablePose3DGizmo.isSelected())
      {
         highlightModelInstance.getRenderables(renderables, pool);
      }
      selectablePose3DGizmo.getVirtualRenderables(renderables, pool);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return representativeReferenceFrame;
   }
}
