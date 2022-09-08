package us.ihmc.gdx.simulation.scs2;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.gizmo.DynamicGDXModel;

import java.util.ArrayList;

public class FrameGDXGraphicsNode
{
   private static final boolean ENABLE_REFERENCE_FRAME_GRAPHICS
         = Boolean.parseBoolean(System.getProperty("frameGDXGraphicsNodeEnableReferenceFrameGraphics", "false"));

   private final ReferenceFrame referenceFrame;
   private ModelInstance coordinateFrame;
   private final ArrayList<FrameGDXNodePart> parts = new ArrayList<>();

   public FrameGDXGraphicsNode(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;

      if (ENABLE_REFERENCE_FRAME_GRAPHICS)
         coordinateFrame = GDXModelBuilder.createCoordinateFrameInstance(0.15);
   }

   public void addModelPart(DynamicGDXModel model, String name, float x, float y, float z)
   {
      parts.add(new FrameGDXNodePart(referenceFrame, model, name, x, y, z));
   }

   public void addModelPart(DynamicGDXModel model, String name)
   {
      parts.add(new FrameGDXNodePart(referenceFrame, model, name));
   }

   public void updatePose()
   {
      for (FrameGDXNodePart part : parts)
      {
         part.update();
      }

      if (ENABLE_REFERENCE_FRAME_GRAPHICS)
         GDXTools.toGDX(referenceFrame.getTransformToRoot(), coordinateFrame.transform);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (FrameGDXNodePart part : parts)
      {
         part.getRenderables(renderables, pool);
      }

      if (ENABLE_REFERENCE_FRAME_GRAPHICS)
         coordinateFrame.getRenderables(renderables, pool);
   }

   public void dispose()
   {
      for (FrameGDXNodePart part : parts)
      {
         part.dispose();
      }
   }

   public void scale(float x, float y ,float z)
   {
      coordinateFrame.transform.scale(x, y, z);
   }
}
