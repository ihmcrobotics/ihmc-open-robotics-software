package us.ihmc.rdx.simulation.scs2;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.gizmo.RDXVisualModelInstance;
import us.ihmc.log.LogTools;

import java.util.concurrent.atomic.AtomicInteger;

public class RDXFrameNodePart
{
   private static final AtomicInteger INDEX = new AtomicInteger();

   private RDXVisualModelInstance modelInstance;
   private final String name;
   private ReferenceFrame modelFrame;

   public RDXFrameNodePart(ReferenceFrame referenceFrame, RDXVisualModelInstance modelInstance, String name, float x, float y, float z)
   {
      this(referenceFrame, modelInstance, name);
      scale(x, y, z);
   }

   public RDXFrameNodePart(ReferenceFrame referenceFrame, RDXVisualModelInstance modelInstance, String name)
   {
      this.modelInstance = modelInstance;
      this.name = name;
      modelFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("modelFrame" + INDEX.getAndIncrement(),
                                                                                     referenceFrame,
                                                                                     modelInstance.getLocalTransform());
   }

   public void update()
   {
      try
      {
         RigidBodyTransform transformToRoot = modelFrame.getTransformToRoot();
         LibGDXTools.toLibGDX(transformToRoot, modelInstance.transform);
      }
      catch (NotARotationMatrixException e) // TODO: Why do we get this sometimes?
      {
         LogTools.warn(e.getMessage());
      }
   }

   public void scale(float x, float y, float z)
   {
      modelInstance.transform.scale(x, y, z);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      modelInstance.getRenderables(renderables, pool);
   }

   public void dispose()
   {

   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }
}
