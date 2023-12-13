package us.ihmc.rdx.simulation.scs2;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.gizmo.RDXVisualModelInstance;

import java.util.ArrayList;

public class RDXFrameGraphicsNode
{
   private final ReferenceFrame referenceFrame;
   private final ArrayList<RDXFrameNodePart> parts = new ArrayList<>();
   private final ModelInstance referenceFrameGraphic;

   public RDXFrameGraphicsNode(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, false);
   }

   public RDXFrameGraphicsNode(ReferenceFrame referenceFrame, boolean createReferenceFrameGraphics)
   {
      this.referenceFrame = referenceFrame;

      referenceFrameGraphic = createReferenceFrameGraphics ? RDXModelBuilder.createCoordinateFrameInstance(0.15) : null;
   }

   public void addModelPart(RDXVisualModelInstance model)
   {
      parts.add(new RDXFrameNodePart(referenceFrame, model));
   }

   public void addModelPart(RDXVisualModelInstance model, String name)
   {
      parts.add(new RDXFrameNodePart(referenceFrame, model, name));
   }

   public void update()
   {
      for (RDXFrameNodePart part : parts)
      {
         part.update();
      }

      if (referenceFrameGraphic != null)
         LibGDXTools.toLibGDX(referenceFrame.getTransformToRoot(), referenceFrameGraphic.transform);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RDXFrameNodePart part : parts)
      {
         part.getRenderables(renderables, pool);
      }
   }

   public void getReferenceFrameRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      referenceFrameGraphic.getRenderables(renderables, pool);
   }

   public ArrayList<RDXFrameNodePart> getParts()
   {
      return parts;
   }
}
