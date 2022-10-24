package us.ihmc.rdx.simulation.bullet;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.math.Vector3;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.rdx.tools.RDXModifiableMultiColorMeshModel;

public class RDXBulletPhysicsDebuggerModel
{
   private ModelInstance modelInstance;
   private final RDXModifiableMultiColorMeshModel modelBuilder = new RDXModifiableMultiColorMeshModel();

   public void addLineGDX(Vector3 from, Vector3 to, Color color)
   {
      modelBuilder.getMeshBuilder().addLine(from.x, from.y, from.z, to.x, to.y, to.z, 0.002f, color);
   }

   public void addLineEuclid(Tuple3DReadOnly from, Tuple3DReadOnly to, Color color)
   {
      modelBuilder.getMeshBuilder().addLine(from, to, 0.002f, color);
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
