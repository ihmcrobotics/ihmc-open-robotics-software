package us.ihmc.rdx.model;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * For us.ihmc.scs2.definition.visual
 */
public class RDXVisualModelInstance extends RDXModelInstance
{
   private final RigidBodyTransform localTransform = new RigidBodyTransform();

   public RDXVisualModelInstance(Model model)
   {
      super(model);
   }

   public RigidBodyTransform getLocalTransform()
   {
      return localTransform;
   }
}
