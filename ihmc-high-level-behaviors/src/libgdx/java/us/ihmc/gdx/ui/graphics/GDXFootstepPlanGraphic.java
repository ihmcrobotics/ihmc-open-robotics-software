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
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.exceptions.OutdatedPolygonException;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.gdx.mesh.GDXIDMappedColorFunction;
import us.ihmc.gdx.mesh.GDXMultiColorMeshBuilder;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.function.Function;

public class GDXFootstepPlanGraphic implements RenderableProvider
{
   private final ModelBuilder modelBuilder = new ModelBuilder();
   GDXMultiColorMeshBuilder meshBuilder = new GDXMultiColorMeshBuilder();

   // visualization options
   private final Function<Integer, Color> colorFunction = new GDXIDMappedColorFunction();
   private final SideDependentList<Color> footstepColors = new SideDependentList<>();
   {
      footstepColors.set(RobotSide.LEFT, new Color(1.0f, 0.0f, 0.0f, 1.0f));
      footstepColors.set(RobotSide.RIGHT, new Color(0.0f, 0.5019608f, 0.0f, 1.0f));
   }
   private final SideDependentList<ConvexPolygon2D> defaultContactPoints = new SideDependentList<>();

   private volatile Runnable buildMeshAndCreateModelInstance = null;

   private ModelInstance modelInstance;
   private Model lastModel;

   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   public GDXFootstepPlanGraphic(SegmentDependentList<RobotSide, ArrayList<Point2D>> controllerFootGroundContactPoints)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D defaultFoothold = new ConvexPolygon2D();
         controllerFootGroundContactPoints.get(robotSide).forEach(defaultFoothold::addVertex);
         defaultFoothold.update();
         defaultContactPoints.put(robotSide, defaultFoothold);
      }
   }

   public GDXFootstepPlanGraphic()
   {
   }

   public void setTransparency(double opacity)
   {
      footstepColors.get(RobotSide.LEFT).a = (float) opacity;
      footstepColors.get(RobotSide.RIGHT).a = (float) opacity;
   }

   public void setColor(RobotSide side, Color color)
   {
      Color sideColor = footstepColors.get(side);
      sideColor.r = color.r;
      sideColor.g = color.g;
      sideColor.b = color.b;
   }

   public void update()
   {
      if (buildMeshAndCreateModelInstance != null)
      {
         buildMeshAndCreateModelInstance.run();
         buildMeshAndCreateModelInstance = null;
      }
   }

   public void generateMeshesAsync(ArrayList<MinimalFootstep> footsteps)
   {
      executorService.clearQueueAndExecute(() -> generateMeshes(footsteps));
   }

   public void generateMeshes(ArrayList<MinimalFootstep> footsteps)
   {
      meshBuilder.clear();

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      ConvexPolygon2D foothold = new ConvexPolygon2D();

      for (int i = 0; i < footsteps.size(); i++)
      {
         MinimalFootstep minimalFootstep = footsteps.get(i);
         Color regionColor = footstepColors.get(minimalFootstep.getSide());

         minimalFootstep.getSolePoseInWorld().get(transformToWorld);
         transformToWorld.appendTranslation(0.0, 0.0, 0.01);

         if (minimalFootstep.getFoothold() != null && !minimalFootstep.getFoothold().isEmpty())
         {
            try
            {
               foothold.set(minimalFootstep.getFoothold());
            }
            catch (OutdatedPolygonException e)
            {
               LogTools.error(e.getMessage() + " See https://github.com/ihmcrobotics/euclid/issues/43");
            }
         }
         else if (defaultContactPoints.containsKey(minimalFootstep.getSide()))
         {
            foothold.set(defaultContactPoints.get(minimalFootstep.getSide()));
         }
         else
         {
            LogTools.error("Must specify default or per footstep foothold");
            throw new RuntimeException("Must specify default or per footstep foothold");
         }

         Point2D[] vertices = new Point2D[foothold.getNumberOfVertices()];
         for (int j = 0; j < vertices.length; j++)
         {
            vertices[j] = new Point2D(foothold.getVertex(j));
         }

         meshBuilder.addMultiLine(transformToWorld, vertices, 0.01, regionColor, true);
         meshBuilder.addPolygon(transformToWorld, foothold, regionColor);
      }
      buildMeshAndCreateModelInstance = () ->
      {
         modelBuilder.begin();
         Mesh mesh = meshBuilder.generateMesh();
         MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL32.GL_TRIANGLES);
         Material material = new Material();
         Texture paletteTexture = new Texture(Gdx.files.classpath("palette.png"));
         material.set(TextureAttribute.createDiffuse(paletteTexture));
         float shade = 0.6f;
         material.set(ColorAttribute.createDiffuse(shade, shade, shade, 1.0f));
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
