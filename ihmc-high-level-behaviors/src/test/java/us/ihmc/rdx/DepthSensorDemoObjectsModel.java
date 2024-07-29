package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.model.Node;
import com.badlogic.gdx.graphics.g3d.utils.MeshPartBuilder;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.graphics.g3d.utils.shapebuilders.*;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;

public class DepthSensorDemoObjectsModel
{
   private final float boxSize = 0.2f;
   private final float distance = 0.5f;
   private final ModelBuilder modelBuilder = new ModelBuilder();

   private int partIndex = 0;

   private final Model model;

   public DepthSensorDemoObjectsModel()
   {
      modelBuilder.begin();
      buildPart(distance, distance, boxSize + 0.1f, Color.GREEN);
      buildPart(-distance, distance, boxSize + 0.1f, Color.DARK_GRAY);
      buildPart(distance, -distance, boxSize + 0.1f, Color.RED);
      buildPart(-distance, -distance, boxSize + 0.1f, Color.ORANGE);
      buildPart(distance / 2.0f, distance / 2.0f, boxSize / 2.0f + 0.1f, Color.BLUE);
      buildPart(-distance / 2.0f, distance / 2.0f, boxSize / 2.0f + 0.1f, Color.BLACK);
      buildPart(distance / 2.0f, -distance / 2.0f, boxSize / 2.0f + 0.1f, Color.WHITE);
      buildPart(-distance / 2.0f, -distance / 2.0f, boxSize / 2.0f + 0.1f, Color.YELLOW);
      model = modelBuilder.end();
   }

   public ModelInstance buildCylinder()
   {
      modelBuilder.begin();
      modelBuilder.node();
      MeshPartBuilder partBuilder = modelBuilder.part("box" + partIndex++,
                                                      GL20.GL_TRIANGLES,
                                                      VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal,
                                                      new Material(PBRColorAttribute.createBaseColorFactor(Color.YELLOW)));
      CylinderShapeBuilder.build(partBuilder, boxSize, boxSize, boxSize, 50);
      return new ModelInstance(modelBuilder.end());
   }

   private void buildPart(float x, float y, float z, Color color)
   {
      Node node = modelBuilder.node();
      node.translation.set(x, y, z);
      MeshPartBuilder partBuilder = modelBuilder.part("box" + partIndex++,
                                                      GL20.GL_TRIANGLES,
                                                      VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal,
                                                      new Material(PBRColorAttribute.createBaseColorFactor(color)));
      if (x > 0)
      {
         if (y > 0)
         {
            BoxShapeBuilder.build(partBuilder, boxSize, boxSize, boxSize);
         }
         else
         {
            SphereShapeBuilder.build(partBuilder, boxSize, boxSize, boxSize, 50, 50);
         }
      }
      else
      {
         if (y > 0)
         {
            ConeShapeBuilder.build(partBuilder, boxSize, boxSize, boxSize, 50);
         }
         else
         {
            CylinderShapeBuilder.build(partBuilder, boxSize, boxSize, boxSize, 50);
         }
      }
   }

   public ModelInstance newInstance()
   {
      return new ModelInstance(model);
   }

   public void dispose()
   {
      model.dispose();
   }
}
