package us.ihmc.gdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;

public class GDXReferenceFrameGraphic implements RenderableProvider
{
   private final RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
   private final FramePose3D framePose3D = new FramePose3D();
   private final ModelInstance coordinateFrameInstance;

   public GDXReferenceFrameGraphic(double length)
   {
      coordinateFrameInstance = GDXModelPrimitives.createCoordinateFrameInstance(length);
   }

   public void setToReferenceFrame(ReferenceFrame referenceFrame)
   {
      framePose3D.setToZero(referenceFrame);
      framePose3D.changeFrame(ReferenceFrame.getWorldFrame());
      framePose3D.get(rigidBodyTransform);
      GDXTools.toGDX(rigidBodyTransform, coordinateFrameInstance.transform);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      coordinateFrameInstance.getRenderables(renderables, pool);
   }

   public void dispose()
   {
      coordinateFrameInstance.model.dispose();
   }

   public ModelInstance getModelInstance()
   {
      return coordinateFrameInstance;
   }
}
