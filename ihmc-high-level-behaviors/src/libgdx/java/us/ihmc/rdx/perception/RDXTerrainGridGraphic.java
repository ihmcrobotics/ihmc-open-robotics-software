package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;

import java.util.ArrayList;

public class RDXTerrainGridGraphic implements RenderableProvider
{
   private final RigidBodyTransform zUpToWorldTransform = new RigidBodyTransform();

   private ArrayList<ModelInstance> zUpGridLines = new ArrayList<>();
   private ArrayList<ModelInstance> worldGridLines = new ArrayList<>();
   private ArrayList<ModelInstance> localGridLines = new ArrayList<>();

   private float zUpGridSize = 4.0f;
   private float worldGridSize = 10.0f;
   private float localGridSize = 4.0f;

   private float zUpGridResolution = 0.1f;
   private float worldGridResolution = 0.1f;
   private float localGridResolution = 0.1f;

   private ImBoolean renderEnabled = new ImBoolean(false);

   public void create()
   {
      createGridLines(zUpGridLines, (float) (Math.PI / 2), zUpGridSize, zUpGridResolution, Color.WHITE);
      createGridLines(zUpGridLines, (float) (Math.PI / 2), zUpGridSize, zUpGridResolution, Color.WHITE);
      createGridLines(worldGridLines, (float) (Math.PI / 2), worldGridSize, worldGridResolution, Color.BLUE);
      createGridLines(worldGridLines, (float) (Math.PI / 2), worldGridSize, worldGridResolution, Color.BLUE);
      createGridLines(localGridLines, (float) (Math.PI / 2), localGridSize, localGridResolution, Color.GRAY);
      createGridLines(localGridLines, (float) (Math.PI / 2), localGridSize, localGridResolution, Color.GRAY);
   }

   private void createGridLines(ArrayList<ModelInstance> gridLines, float yaw, float size, float resolution, Color color)
   {
      Point3D offset = new Point3D();
      for (int i = 0; i < (int) (size / resolution); i++)
      {
         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         rigidBodyTransform.getTranslation().set(-size / 2 + i * resolution, 0.0, 0.0);
         rigidBodyTransform.getRotation().setYawPitchRoll(yaw, Math.PI / 2, 0);
         ModelInstance cylinder = RDXModelBuilder.createCylinder(4.0f, 0.02f, offset, color);
         LibGDXTools.toLibGDX(rigidBodyTransform, cylinder.transform);
         gridLines.add(cylinder);
      }
   }

   public void update(RigidBodyTransform zUpToWorldTransform)
   {
      this.zUpToWorldTransform.set(zUpToWorldTransform);
      RigidBodyTransform currentTransform = new RigidBodyTransform();
      for (ModelInstance model : zUpGridLines)
      {
         LibGDXTools.toEuclid(model.transform, currentTransform);
         currentTransform.multiply(zUpToWorldTransform);
         LibGDXTools.toLibGDX(currentTransform, model.transform);
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox("Render Grid", renderEnabled);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderEnabled.get())
      {
         for (ModelInstance instance : zUpGridLines)
            instance.getRenderables(renderables, pool);

         for (ModelInstance instance : worldGridLines)
            instance.getRenderables(renderables, pool);

         for (ModelInstance instance : localGridLines)
            instance.getRenderables(renderables, pool);
      }
   }
}
