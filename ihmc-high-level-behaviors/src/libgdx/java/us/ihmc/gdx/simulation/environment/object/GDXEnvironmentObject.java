package us.ihmc.gdx.simulation.environment.object;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.gdx.simulation.environment.GDXModelInstance;

public class GDXEnvironmentObject
{
   private final Model model;
   private final Pose3D pose = new Pose3D();

   private final GDXModelInstance modelInstance;

   protected GDXEnvironmentObject()
   {
      this.model = getModel();
      modelInstance = new GDXModelInstance(model);
   }

   public GDXEnvironmentObject(Model model)
   {
      this.model = model;
      modelInstance = new GDXModelInstance(model);
   }

   // For subclasses to implement
   protected Model getModel()
   {
      return model;
   }

   public GDXModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public GDXEnvironmentObject duplicate()
   {
      return new GDXEnvironmentObject(model);
   }
}
