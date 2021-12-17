package us.ihmc.gdx.ui.graphics;

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
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.exceptions.OutdatedPolygonException;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.gdx.GDX3DSituatedText;
import us.ihmc.gdx.mesh.GDXIDMappedColorFunction;
import us.ihmc.gdx.mesh.GDXMultiColorMeshBuilder;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.gdx.tools.GDXTools;
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
      footstepColors.set(RobotSide.LEFT, new Color(GDXFootstepGraphic.LEFT_FOOT_RED_COLOR));
      footstepColors.set(RobotSide.RIGHT, new Color(GDXFootstepGraphic.RIGHT_FOOT_GREEN_COLOR));
   }
   private final SideDependentList<ConvexPolygon2D> defaultContactPoints = new SideDependentList<>();
   private volatile Runnable buildMeshAndCreateModelInstance = null;
   private ModelInstance modelInstance;
   private Model lastModel;
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ArrayList<GDX3DSituatedText> textRenderables = new ArrayList<>();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final ReferenceFrame footstepFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("footstepFrame",
                                                                                                                ReferenceFrame.getWorldFrame(),
                                                                                                                tempTransform);
   private final FramePose3D textFramePose = new FramePose3D();

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

   public void clearAsync()
   {
      generateMeshesAsync(new ArrayList<>());
   }

   public void generateMeshesAsync(ArrayList<MinimalFootstep> footsteps)
   {
      executorService.clearQueueAndExecute(() -> generateMeshes(footsteps));
   }

   public void clear()
   {
      generateMeshes(new ArrayList<>());
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
         transformToWorld.appendTranslation(0.0, 0.0, 0.001);

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
         // This can't be done outside the libGDX thread. TODO: Consider using Gdx.app.postRunnable
         textRenderables.clear();
         for (int i = 0; i < footsteps.size(); i++)
         {
            MinimalFootstep minimalFootstep = footsteps.get(i);
            GDX3DSituatedText footstepIndexText = new GDX3DSituatedText("" + i);
            minimalFootstep.getSolePoseInWorld().get(tempTransform);
            double textHeight = 0.08;
            footstepFrame.update();
            textFramePose.setToZero(footstepFrame);
            textFramePose.getOrientation().prependYawRotation(-Math.PI / 2.0);
            textFramePose.getPosition().addZ(0.01);
            textFramePose.getPosition().addY(textHeight / 4.0);
            textFramePose.getPosition().addX(-textHeight / 2.0);
            textFramePose.changeFrame(ReferenceFrame.getWorldFrame());
            GDXTools.toGDX(textFramePose, tempTransform, footstepIndexText.getModelInstance().transform);
            footstepIndexText.scale((float) textHeight);
            textRenderables.add(footstepIndexText);

            if (minimalFootstep.getDescription() != null && !minimalFootstep.getDescription().isEmpty())
            {
               GDX3DSituatedText footstepListDescriptionText = new GDX3DSituatedText(minimalFootstep.getDescription());
               textFramePose.changeFrame(footstepFrame);
               textFramePose.getPosition().subY(0.12);
               textFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               GDXTools.toGDX(textFramePose, tempTransform, footstepListDescriptionText.getModelInstance().transform);
               footstepListDescriptionText.scale((float) textHeight);
               textRenderables.add(footstepListDescriptionText);
            }
         }

         modelBuilder.begin();
         Mesh mesh = meshBuilder.generateMesh();
         MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
         Material material = new Material();
         Texture paletteTexture = GDXMultiColorMeshBuilder.loadPaletteTexture();
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
         for (GDX3DSituatedText textRenderable : textRenderables)
         {
            textRenderable.getRenderables(renderables, pool);
         }
      }
   }

   public void destroy()
   {
      executorService.destroy();
   }
}
