package us.ihmc.gdx.visualizers;

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
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.gdx.mesh.GDXMeshGraphicTools;
import us.ihmc.gdx.mesh.GDXMultiColorMeshBuilder;
import us.ihmc.gdx.mesh.GDXIDMappedColorFunction;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.function.Function;

public class GDXPlanarRegionsGraphic implements RenderableProvider
{
   private final ModelBuilder modelBuilder = new ModelBuilder();

   // visualization options
   private Function<Integer, Color> colorFunction = new GDXIDMappedColorFunction();
   private boolean drawAreaText = false;
   private boolean drawBoundingBox = false;
   private boolean drawNormal;

   private volatile Runnable buildMeshAndCreateModelInstance = null;

   private ModelInstance modelInstance;
   private Model lastModel;

   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   public void clear()
   {
      generateMeshes(new PlanarRegionsList());
   }

   public void generateFlatGround()
   {
      generateMeshes(PlanarRegionsList.flatGround(20.0));
   }

   public void update()
   {
      if (buildMeshAndCreateModelInstance != null)
      {
         buildMeshAndCreateModelInstance.run();
         buildMeshAndCreateModelInstance = null;
      }
   }

   public void generateMeshesAsync(PlanarRegionsList planarRegionsList)
   {
      executorService.clearQueueAndExecute(() -> generateMeshes(planarRegionsList));
   }

   public synchronized void generateMeshes(PlanarRegionsList planarRegionsList)
   {
      ArrayList<GDXMultiColorMeshBuilder> meshBuilders = new ArrayList<>();
      for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
      {
         GDXMultiColorMeshBuilder meshBuilder = new GDXMultiColorMeshBuilder();
         meshBuilders.add(meshBuilder);
         singleRegionMeshBuilder(planarRegion, meshBuilder); // should do in parallel somehow?
      }
      buildMeshAndCreateModelInstance = () ->
      {
         modelBuilder.begin();
         Material material = new Material();
         Texture paletteTexture = GDXMultiColorMeshBuilder.loadPaletteTexture();
         material.set(TextureAttribute.createDiffuse(paletteTexture));
         material.set(ColorAttribute.createDiffuse(new Color(0.7f, 0.7f, 0.7f, 1.0f)));

         for (GDXMultiColorMeshBuilder meshBuilder : meshBuilders)
         {
            Mesh mesh = meshBuilder.generateMesh();
            MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL32.GL_TRIANGLES);
            modelBuilder.part(meshPart, material);
         }

         if (lastModel != null)
            lastModel.dispose();

         lastModel = modelBuilder.end();
         modelInstance = new ModelInstance(lastModel); // TODO: Clean up garbage and look into reusing the Model
      };
   }

   private void singleRegionMeshBuilder(PlanarRegion planarRegion, GDXMultiColorMeshBuilder meshBuilder)
   {
      RigidBodyTransform transformToWorld = planarRegion.getTransformToWorldCopy();
      Color color = colorFunction.apply(planarRegion.getRegionId());

      meshBuilder.addMultiLine(transformToWorld, planarRegion.getConcaveHull(), 0.01, color, true);

      double totalArea = 0.0;
      for (ConvexPolygon2D convexPolygon : planarRegion.getConvexPolygons())
      {
         meshBuilder.addPolygon(transformToWorld, convexPolygon, color);

         totalArea += convexPolygon.getArea();
      }

//      LabelGraphic sizeLabel = null;
//      if (drawAreaText)
//      {
//         sizeLabel = new LabelGraphic(FormattingTools.getFormattedToSignificantFigures(totalArea, 3));
//         sizeLabel.getPose().appendTransform(transformToWorld);
//         sizeLabel.update();
//      }

      if (drawBoundingBox)
      {
         GDXMeshGraphicTools.drawBoxEdges(meshBuilder, PlanarRegionTools.getLocalBoundingBox3DInWorld(planarRegion, 0.1), 0.005, color);
      }

      if (drawNormal)
      {
         Point3DReadOnly centroid = PlanarRegionTools.getCentroid3DInWorld(planarRegion);

         double length = 0.07;
         double radius = 0.004;
         double cylinderToConeLengthRatio = 0.8;
         double coneDiameterMultiplier = 1.8;
         GDXMeshGraphicTools.drawArrow(meshBuilder,
                                      centroid,
                                      transformToWorld.getRotation(),
                                      length,
                                      radius,
                                      cylinderToConeLengthRatio,
                                      coneDiameterMultiplier, color);
      }


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

   public void setDrawAreaText(boolean drawAreaText)
   {
      this.drawAreaText = drawAreaText;
   }

   public void setDrawBoundingBox(boolean drawBoundingBox)
   {
      this.drawBoundingBox = drawBoundingBox;
   }

   public void setDrawNormal(boolean drawNormal)
   {
      this.drawNormal = drawNormal;
   }

   public void setColorFunction(Function<Integer, Color> colorFunction)
   {
      this.colorFunction = colorFunction;
   }
}
