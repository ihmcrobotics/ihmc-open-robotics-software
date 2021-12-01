package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.collidables.GDXCollidableCoordinateFrame;
import us.ihmc.gdx.ui.graphics.GDXReferenceFrameGraphic;
import us.ihmc.gdx.ui.interactable.GDXScaledModelInstance;

public class GDXInteractableReferenceFrame
{
   private ReferenceFrame referenceFrame;
   private final FramePose3D tempFramePose = new FramePose3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private GDXCollidableCoordinateFrame collidableCoordinateFrame;
   private GDXReferenceFrameGraphic referenceFrameGraphic;
   private GDXReferenceFrameGraphic highlightReferenceFrameGraphic;
   private GDXScaledModelInstance scaledHighlightReferenceFrameGraphic;
   private boolean mouseCollidesWithFrame;
   private GDXSelectablePose3DGizmo selectablePose3DGizmo;

   public void create(ReferenceFrame referenceFrame, double length, FocusBasedGDXCamera camera3D)
   {
      this.referenceFrame = referenceFrame;
      collidableCoordinateFrame = new GDXCollidableCoordinateFrame(length);
      referenceFrameGraphic = new GDXReferenceFrameGraphic(length);
      highlightReferenceFrameGraphic = new GDXReferenceFrameGraphic(length);
      scaledHighlightReferenceFrameGraphic = new GDXScaledModelInstance(highlightReferenceFrameGraphic.getModelInstance());
      scaledHighlightReferenceFrameGraphic.scale(1.01);
      GDXTools.setTransparency(highlightReferenceFrameGraphic.getModelInstance(), 0.5f);
      selectablePose3DGizmo = new GDXSelectablePose3DGizmo(referenceFrame);
      selectablePose3DGizmo.create(camera3D);
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      Line3DReadOnly pickRay = input.getPickRayInWorld();
      mouseCollidesWithFrame = !Double.isNaN(collidableCoordinateFrame.intersect(selectablePose3DGizmo.getPoseGizmo().getReferenceFrame(), pickRay));

      selectablePose3DGizmo.process3DViewInput(input, mouseCollidesWithFrame);
      if (selectablePose3DGizmo.isSelected())
      {
         tempFramePose.setToZero(selectablePose3DGizmo.getPoseGizmo().getReferenceFrame());
         tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
         tempFramePose.get(tempTransform);
      }

      GDXTools.toGDX(tempTransform, referenceFrameGraphic.getModelInstance().transform);
      GDXTools.toGDX(tempTransform, highlightReferenceFrameGraphic.getModelInstance().transform);
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      referenceFrameGraphic.getRenderables(renderables, pool);
      if (mouseCollidesWithFrame || selectablePose3DGizmo.isSelected())
      {
         highlightReferenceFrameGraphic.getRenderables(renderables, pool);
      }
      selectablePose3DGizmo.getVirtualRenderables(renderables, pool);
   }

   public GDXSelectablePose3DGizmo getSelectablePose3DGizmo()
   {
      return selectablePose3DGizmo;
   }
}
