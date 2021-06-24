package us.ihmc.gdx.ui.graphics;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.opengl.GL32;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.gdx.mesh.GDXMultiColorMeshBuilder;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PathTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.List;

public class GDXBodyPathPlanGraphic implements RenderableProvider
{
   private static final double LINE_THICKNESS = 0.025;
   private final float startColorHue = (float) javafx.scene.paint.Color.GREEN.getHue();
   private final float goalColorHue = (float) javafx.scene.paint.Color.RED.getHue();

   private volatile Runnable buildMeshAndCreateModelInstance = null;

   private final ModelBuilder modelBuilder = new ModelBuilder();
   private ModelInstance modelInstance;
   private Model lastModel;

   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   public void update()
   {
      if (buildMeshAndCreateModelInstance != null)
      {
         buildMeshAndCreateModelInstance.run();
         buildMeshAndCreateModelInstance = null;
      }
   }

   public void clear()
   {
      generateMeshes(new ArrayList<>());
   }

   /**
    * To process in parallel.
    */
   public void generateMeshesAsync(List<? extends Pose3DReadOnly> bodyPath)
   {
      executorService.submit(() -> generateMeshes(bodyPath));
   }

   public void generateMeshes(List<? extends Pose3DReadOnly> bodyPath)
   {
      GDXMultiColorMeshBuilder meshBuilder = new GDXMultiColorMeshBuilder();

      double totalPathLength = PathTools.computePosePathLength(bodyPath);
      double currentLength = 0.0;

      // TODO: Draw orientation somehow
      for (int segmentIndex = 0; segmentIndex < bodyPath.size() - 1; segmentIndex++)
      {
         Point3DReadOnly lineStart = bodyPath.get(segmentIndex).getPosition();
         Point3DReadOnly lineEnd = bodyPath.get(segmentIndex + 1).getPosition();

         float lineStartHue = (float) EuclidCoreTools.interpolate(startColorHue, goalColorHue, currentLength / totalPathLength);
         currentLength += lineStart.distance(lineEnd);
         float lineEndHue = (float) EuclidCoreTools.interpolate(startColorHue, goalColorHue, currentLength / totalPathLength);
         meshBuilder.addLine(lineStart, lineEnd, LINE_THICKNESS, new Color().fromHsv(lineStartHue, 1.0f, 0.5f), new Color().fromHsv(lineEndHue, 1.0f, 1.0f));
      }

      buildMeshAndCreateModelInstance = () ->
      {
         modelBuilder.begin();
         Mesh mesh = meshBuilder.generateMesh();
         MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL32.GL_TRIANGLES);
         com.badlogic.gdx.graphics.g3d.Material material = new Material();
         Texture paletteTexture = new Texture(Gdx.files.classpath("palette.png"));
         material.set(TextureAttribute.createDiffuse(paletteTexture));
         material.set(ColorAttribute.createDiffuse(new com.badlogic.gdx.graphics.Color(0.7f, 0.7f, 0.7f, 1.0f)));
         modelBuilder.part(meshPart, material);

         if (lastModel != null)
            lastModel.dispose();

         lastModel = modelBuilder.end();
         modelInstance = new ModelInstance(lastModel); // TODO: Clean up garbage and look into reusing the Model
      };
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      // sync over current and add
      if (modelInstance != null)
      {
         modelInstance.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      executorService.destroy();
   }
}
