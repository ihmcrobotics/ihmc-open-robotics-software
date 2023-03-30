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

   private final RDXVisualModelInstance modelInstance;
   // The name part is just for optional debugging
   private String name;
   private final ReferenceFrame partFrame;

   public RDXFrameNodePart(ReferenceFrame referenceFrame, RDXVisualModelInstance modelInstance)
   {
      this(referenceFrame, modelInstance, "");
   }

   public RDXFrameNodePart(ReferenceFrame referenceFrame, RDXVisualModelInstance modelInstance, String name)
   {
      this.modelInstance = modelInstance;
      this.name = name;
      partFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("partFrame" + INDEX.getAndIncrement(),
                                                                                    referenceFrame,
                                                                                    modelInstance.getLocalTransform());
   }

   public void update()
   {
      try
      {
         RigidBodyTransform transformToRoot = partFrame.getTransformToRoot();
         LibGDXTools.toLibGDX(transformToRoot, modelInstance.transform);
      }
      catch (NotARotationMatrixException e) // TODO: Why do we get this sometimes?
      {
         LogTools.warn(e.getMessage());
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      modelInstance.getRenderables(renderables, pool);
   }

   public RDXVisualModelInstance getModelInstance()
   {
      return modelInstance;
   }
}
