package us.ihmc.graphics3DAdapter;

import java.util.concurrent.Callable;

import javax.media.j3d.Transform3D;

import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;

public class GraphicsTransformUpdate implements Callable<Object>
{
   private Transform3D transform;
   private Graphics3DNode node;

   public GraphicsTransformUpdate(Graphics3DNode node, Transform3D transform)
   {
      this.node = node;
      this.transform = transform;
   }

   public Object call() throws Exception
   {
      node.setTransform(transform);

      return null;
   }
}
