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
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.collidables.GDXCoordinateFrameIntersection;
import us.ihmc.gdx.ui.graphics.GDXReferenceFrameGraphic;
import us.ihmc.gdx.ui.interactable.GDXModelInstanceScaler;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class GDXInteractableReferenceFrame
{
   private ReferenceFrame representativeReferenceFrame;
   private final FramePose3D tempFramePose = new FramePose3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private RigidBodyTransform transformToParent;
   private GDXReferenceFrameGraphic referenceFrameGraphic;
   private GDXReferenceFrameGraphic highlightReferenceFrameGraphic;
   private GDXCoordinateFrameIntersection coordinateFrameIntersection;
   private GDXModelInstanceScaler highlightReferenceFrameGraphicScaler;
   private boolean isCoordinateFramePickSelected;
   private GDXSelectablePose3DGizmo selectablePose3DGizmo;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();

   public void create(ReferenceFrame parentFrame, double length, GDX3DPanel panel3D)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      ReferenceFrame referenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(parentFrame, transform);
      create(referenceFrame, transform, length, panel3D);
   }

   public void create(ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify, double length, GDX3DPanel panel3D)
   {
      representativeReferenceFrame = referenceFrameToRepresent;
      transformToParent = transformToParentToModify;
      referenceFrameGraphic = new GDXReferenceFrameGraphic(length);
      highlightReferenceFrameGraphic = new GDXReferenceFrameGraphic(length);
      coordinateFrameIntersection = new GDXCoordinateFrameIntersection(length);
      highlightReferenceFrameGraphicScaler = new GDXModelInstanceScaler(highlightReferenceFrameGraphic);
      highlightReferenceFrameGraphicScaler.scale(1.01);
      GDXTools.setTransparency(highlightReferenceFrameGraphic, 0.5f);
      selectablePose3DGizmo = new GDXSelectablePose3DGizmo(representativeReferenceFrame, transformToParent);
      selectablePose3DGizmo.create(panel3D);
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

      GDXTools.toGDX(tempTransform, referenceFrameGraphic.transform);
      GDXTools.toGDX(tempTransform, highlightReferenceFrameGraphic.transform);
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

   public GDXSelectablePose3DGizmo getSelectablePose3DGizmo()
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
