package us.ihmc.gdx.simulation.bullet;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.math.Vector3;
import us.ihmc.gdx.tools.GDXModifiableMultiColorMeshModel;

public class GDXBulletPhysicsDebuggerModel
{
   private ModelInstance modelInstance;
   private final GDXModifiableMultiColorMeshModel modelBuilder = new GDXModifiableMultiColorMeshModel();

   public void addLine(Vector3 from, Vector3 to, Vector3 color)
   {
      modelBuilder.getMeshBuilder().addLine(from.x, from.y, from.z, to.x, to.y, to.z, 0.002f, new Color(color.x, color.y, color.z, 1.0f));
   }

   public void begin()
   {
      modelBuilder.begin();
   }

   public void end()
   {
      modelBuilder.end();
      modelInstance = new ModelInstance(modelBuilder.getModel()); // TODO: Allocation free?
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }
}
