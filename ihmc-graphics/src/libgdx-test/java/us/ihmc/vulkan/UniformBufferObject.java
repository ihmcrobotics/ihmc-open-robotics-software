package us.ihmc.vulkan;

import org.ejml.data.DMatrix4x4;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class UniformBufferObject
{
   public static final int SIZEOF = 3 * 16 * Float.BYTES;

   private final RigidBodyTransform model = new RigidBodyTransform();
   private final RigidBodyTransform view = new RigidBodyTransform();
   private final DMatrix4x4 projection = new DMatrix4x4();

   public RigidBodyTransform getModel()
   {
      return model;
   }

   public RigidBodyTransform getView()
   {
      return view;
   }

   public DMatrix4x4 getProjection()
   {
      return projection;
   }
}
