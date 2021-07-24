package us.ihmc.gdx.simulation.scs2;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.gizmo.DynamicGDXModel;

import java.util.ArrayList;

public class FrameGDXNode
{
   private final ReferenceFrame referenceFrame;
   private final ModelInstance coordinateFrame;
   private final ArrayList<FrameGDXNodePart> parts = new ArrayList<>();

   public FrameGDXNode(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      coordinateFrame = GDXModelPrimitives.createCoordinateFrameInstance(0.15);
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

      GDXTools.toGDX(referenceFrame.getTransformToRoot(), coordinateFrame.transform);
   }

   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (FrameGDXNodePart part : parts)
      {
         part.getRenderables(renderables, pool);
      }

      coordinateFrame.getRenderables(renderables, pool);
   }
}
