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
   private static final boolean ENABLE_REFERENCE_FRAME_GRAPHICS
         = Boolean.parseBoolean(System.getProperty("frameGDXGraphicsNodeEnableReferenceFrameGraphics", "false"));

   private final ReferenceFrame referenceFrame;
   private ModelInstance coordinateFrame;
   private final ArrayList<RDXFrameNodePart> parts = new ArrayList<>();

   public RDXFrameGraphicsNode(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;

      if (ENABLE_REFERENCE_FRAME_GRAPHICS)
         coordinateFrame = RDXModelBuilder.createCoordinateFrameInstance(0.15);
   }

   public void addModelPart(RDXVisualModelInstance model, String name, float x, float y, float z)
   {
      parts.add(new RDXFrameNodePart(referenceFrame, model, name, x, y, z));
   }

   public void addModelPart(RDXVisualModelInstance model, String name)
   {
      parts.add(new RDXFrameNodePart(referenceFrame, model, name));
   }

   public void updatePose()
   {
      for (RDXFrameNodePart part : parts)
      {
         part.update();
      }

      if (ENABLE_REFERENCE_FRAME_GRAPHICS)
         LibGDXTools.toGDX(referenceFrame.getTransformToRoot(), coordinateFrame.transform);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RDXFrameNodePart part : parts)
      {
         part.getRenderables(renderables, pool);
      }

      if (ENABLE_REFERENCE_FRAME_GRAPHICS)
         coordinateFrame.getRenderables(renderables, pool);
   }

   public void dispose()
   {
      for (RDXFrameNodePart part : parts)
      {
         part.dispose();
      }
   }

   public void scale(float x, float y ,float z)
   {
      coordinateFrame.transform.scale(x, y, z);
   }

   public ArrayList<RDXFrameNodePart> getParts()
   {
      return parts;
   }
}
