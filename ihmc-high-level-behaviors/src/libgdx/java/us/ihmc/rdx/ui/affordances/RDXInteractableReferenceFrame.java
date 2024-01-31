package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.collidables.RDXCoordinateFrameIntersection;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class RDXInteractableReferenceFrame
{
   private ReferenceFrame representativeReferenceFrame;
   private final FramePose3D tempFramePose = new FramePose3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private RigidBodyTransform transformToParent;
   private RDXReferenceFrameGraphic referenceFrameGraphic;
   private RDXReferenceFrameGraphic highlightReferenceFrameGraphic;
   private RDXCoordinateFrameIntersection coordinateFrameIntersection;
   private boolean isCoordinateFramePickSelected;
   private RDXSelectablePose3DGizmo selectablePose3DGizmo;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();

   public void create(ReferenceFrame parentFrame, double length, RDX3DPanel panel3D)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      ReferenceFrame referenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(parentFrame, transform);
      create(referenceFrame, transform, length, panel3D);
   }

   public void create(ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify, double length, RDX3DPanel panel3D)
   {
      representativeReferenceFrame = referenceFrameToRepresent;
      transformToParent = transformToParentToModify;
      referenceFrameGraphic = new RDXReferenceFrameGraphic(length);
      highlightReferenceFrameGraphic = new RDXReferenceFrameGraphic(length + 0.001);
      coordinateFrameIntersection = new RDXCoordinateFrameIntersection(length);
      LibGDXTools.setOpacity(highlightReferenceFrameGraphic, 0.5f);
      selectablePose3DGizmo = new RDXSelectablePose3DGizmo(representativeReferenceFrame, transformToParent);
      selectablePose3DGizmo.create(panel3D);
   }

   public void createAndSetupDefault(RDXBaseUI baseUI, ReferenceFrame parentFrame, double length)
   {
      create(parentFrame, length, baseUI.getPrimary3DPanel());
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(this::calculate3DViewPick);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::process3DViewInput);
      baseUI.getPrimaryScene().addRenderableProvider(this::getVirtualRenderables);
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      selectablePose3DGizmo.calculate3DViewPick(input);

      Line3DReadOnly pickRay = input.getPickRayInWorld();
      double closestCollisionDistance = coordinateFrameIntersection.intersect(representativeReferenceFrame, pickRay);
      if (!Double.isNaN(closestCollisionDistance))
      {
         pickResult.setDistanceToCamera(closestCollisionDistance);
         input.addPickResult(pickResult);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      isCoordinateFramePickSelected = pickResult == input.getClosestPick();

      selectablePose3DGizmo.process3DViewInput(input, isCoordinateFramePickSelected);
      tempFramePose.setToZero(representativeReferenceFrame);
      tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      tempFramePose.get(tempTransform);

      LibGDXTools.toLibGDX(tempTransform, referenceFrameGraphic.transform);
      LibGDXTools.toLibGDX(tempTransform, highlightReferenceFrameGraphic.transform);
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      referenceFrameGraphic.getRenderables(renderables, pool);
      if (isCoordinateFramePickSelected || selectablePose3DGizmo.isSelected())
      {
         highlightReferenceFrameGraphic.getRenderables(renderables, pool);
      }
      selectablePose3DGizmo.getVirtualRenderables(renderables, pool);
   }

   public RDXSelectablePose3DGizmo getSelectablePose3DGizmo()
   {
      return selectablePose3DGizmo;
   }

   public ReferenceFrame getRepresentativeReferenceFrame()
   {
      return representativeReferenceFrame;
   }

   public RigidBodyTransform getTransformToParent()
   {
      return transformToParent;
   }
}
