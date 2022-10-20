package us.ihmc.rdx.ui.gizmo;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.tools.GDXModelInstance;

/**
 * For us.ihmc.scs2.definition.visual
 */
public class GDXVisualModelInstance extends GDXModelInstance
{
   private final RigidBodyTransform localTransform = new RigidBodyTransform();

   public GDXVisualModelInstance(Model model)
   {
      super(model);
   }

   public RigidBodyTransform getLocalTransform()
   {
      return localTransform;
   }
}
