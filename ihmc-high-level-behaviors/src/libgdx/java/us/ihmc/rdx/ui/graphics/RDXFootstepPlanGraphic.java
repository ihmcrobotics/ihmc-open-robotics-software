package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepQueueStatusMessage;
import us.ihmc.behaviors.tools.MinimalFootstep;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.exceptions.OutdatedPolygonException;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.RDX3DSituatedText;
import us.ihmc.rdx.mesh.RDXIDMappedColorFunction;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.function.Function;

public class RDXFootstepPlanGraphic implements RenderableProvider
{
   private final ModelBuilder modelBuilder = new ModelBuilder();
   RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();
   // visualization options
   private final Function<Integer, Color> colorFunction = new RDXIDMappedColorFunction();
   private final SideDependentList<Color> footstepColors = new SideDependentList<>();
   {
      footstepColors.set(RobotSide.LEFT, new Color(RDXFootstepGraphic.LEFT_FOOT_RED_COLOR));
      footstepColors.set(RobotSide.RIGHT, new Color(RDXFootstepGraphic.RIGHT_FOOT_GREEN_COLOR));
   }
   private final SideDependentList<ConvexPolygon2D> defaultContactPoints = new SideDependentList<>();
   private volatile Runnable buildMeshAndCreateModelInstance = null;

   private ModelInstance modelInstance;
   private Model lastModel;

   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ArrayList<RDX3DSituatedText> textRenderables = new ArrayList<>();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final ReferenceFrame footstepFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("footstepFrame",
                                                                                                                ReferenceFrame.getWorldFrame(),
                                                                                                                tempTransform);
   private final FramePose3D textFramePose = new FramePose3D();
   private boolean isEmpty = true;

   public RDXFootstepPlanGraphic(SegmentDependentList<RobotSide, ArrayList<Point2D>> controllerFootGroundContactPoints)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D defaultFoothold = new ConvexPolygon2D();
         controllerFootGroundContactPoints.get(robotSide).forEach(defaultFoothold::addVertex);
         defaultFoothold.update();
         defaultContactPoints.put(robotSide, defaultFoothold);
      }
   }

   public RDXFootstepPlanGraphic()
   {
   }

   public void setOpacity(double opacity)
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

   public void generateMeshesAsync(FootstepQueueStatusMessage queueStatusMessage, String description)
   {
      generateMeshesAsync(MinimalFootstep.convertFootstepQueueMessage(queueStatusMessage, description));
   }

   public void generateMeshesAsync(FootstepDataListMessage footstepDataListMessage, String description)
   {
      generateMeshesAsync(MinimalFootstep.convertFootstepDataListMessage(footstepDataListMessage, description));
   }

   public void generateMeshesAsync(ArrayList<MinimalFootstep> footsteps)
   {
      executorService.clearQueueAndExecute(() -> generateMeshes(footsteps));
   }

   public void clear()
   {
      // this prevents generating empty plans like crazy which is expensive
      if (isEmpty)
         return;

      generateMeshes(new ArrayList<>());
   }

   public void generateMeshes(ArrayList<MinimalFootstep> footsteps)
   {
      // this prevents generating empty plans like crazy which is expensive
      if (isEmpty && footsteps.isEmpty())
         return;
      isEmpty = footsteps.isEmpty();

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
         // This can't be done outside the libGDX thread. TODO: Consider using Gdx.app.postRunnable
         textRenderables.clear();
         for (int i = 0; i < footsteps.size(); i++)
         {
            MinimalFootstep minimalFootstep = footsteps.get(i);
            float textHeight = 0.08f;
            RDX3DSituatedText footstepIndexText = new RDX3DSituatedText("" + i, textHeight);
            minimalFootstep.getSolePoseInWorld().get(tempTransform);
            footstepFrame.update();
            textFramePose.setToZero(footstepFrame);
            textFramePose.getOrientation().prependYawRotation(-Math.PI / 2.0);
            textFramePose.getPosition().addZ(0.01);
            textFramePose.getPosition().addY(textHeight / 4.0);
            textFramePose.getPosition().addX(-textHeight / 2.0);
            textFramePose.changeFrame(ReferenceFrame.getWorldFrame());
            LibGDXTools.toLibGDX(textFramePose, tempTransform, footstepIndexText.getModelTransform());
            textRenderables.add(footstepIndexText);

            if (minimalFootstep.getDescription() != null && !minimalFootstep.getDescription().isEmpty())
            {
               RDX3DSituatedText footstepListDescriptionText = new RDX3DSituatedText(minimalFootstep.getDescription(), textHeight);
               textFramePose.changeFrame(footstepFrame);
               textFramePose.getPosition().subY(0.12);
               textFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               LibGDXTools.toLibGDX(textFramePose, tempTransform, footstepListDescriptionText.getModelTransform());
               textRenderables.add(footstepListDescriptionText);
            }
         }

         if (lastModel != null)
            lastModel.dispose();

         lastModel = RDXModelBuilder.buildModelFromMesh(modelBuilder, meshBuilder);
         LibGDXTools.setOpacity(lastModel, footstepColors.get(RobotSide.LEFT).a);
         modelInstance = new ModelInstance(lastModel); // TODO: Clean up garbage and look into reusing the Model
      };
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modelInstance != null)
      {
         modelInstance.getRenderables(renderables, pool);
         for (RDX3DSituatedText textRenderable : textRenderables)
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
