package us.ihmc.gdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.model.Node;
import com.badlogic.gdx.graphics.g3d.utils.MeshPartBuilder;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.graphics.g3d.utils.shapebuilders.BoxShapeBuilder;

public class BoxesDemoModel
{
   private final float boxSize = 1.0f;
   private final float distance = 5.0f;
   private final ModelBuilder modelBuilder = new ModelBuilder();

   private int partIndex = 0;

   private final Model model;

   public BoxesDemoModel()
   {
      modelBuilder.begin();
      buildBoxPart(distance, distance, distance, Color.GREEN);
      buildBoxPart(-distance, distance, distance, Color.DARK_GRAY);
      buildBoxPart(distance, -distance, distance, Color.RED);
      buildBoxPart(-distance, -distance, distance, Color.ORANGE);
      buildBoxPart(distance, distance, -distance, Color.BLUE);
      buildBoxPart(-distance, distance, -distance, Color.BLACK);
      buildBoxPart(distance, -distance, -distance, Color.WHITE);
      buildBoxPart(-distance, -distance, -distance, Color.YELLOW);
      model = modelBuilder.end();
   }

   private void buildBoxPart(float x, float y, float z, Color color)
   {
      Node node = modelBuilder.node();
      node.translation.set(x, y, z);
      MeshPartBuilder partBuilder = modelBuilder.part("box" + partIndex++,
                                                      GL20.GL_TRIANGLES,
                                                      VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal,
                                                      new Material(ColorAttribute.createDiffuse(color)));
      BoxShapeBuilder.build(partBuilder, boxSize, boxSize, boxSize);
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
