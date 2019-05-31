package us.ihmc.quadrupedUI.graphics;

import controller_msgs.msg.dds.QuadrupedTimedStepListMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXVisualizers.PrivateAnimationTimer;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class ManualStepPlanGraphic extends Group
{
   private static final double RADIUS = 0.02;
   private static final double zOffset = 0.01;
   private static final QuadrantDependentList<Color> solutionFootstepColors = new QuadrantDependentList<>(Color.BLUE, Color.ORANGE, Color.DARKBLUE, Color.DARKORANGE);

   private final MeshView meshView = new MeshView();
   private final PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::handle);
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);
   private Mesh mesh;
   private Material material;

   public ManualStepPlanGraphic()
   {
      getChildren().addAll(meshView);

      animationTimer.start();
   }

   /**
    * To process in parallel.
    */
   public void generateMeshesAsynchronously(QuadrupedTimedStepListMessage plan)
   {
      executorService.submit(() -> {
         LogTools.debug("Received pawstep plan containing {} steps", plan.getQuadrupedStepList().size());
         generateMeshes(plan);
      });
   }

   public void generateMeshes(QuadrupedTimedStepListMessage message)
   {
      meshBuilder.clear();

      QuadrantDependentList<Color> colors = solutionFootstepColors;

      FramePoint3D footPosition = new FramePoint3D();

      for (int i = 0; i < message.getQuadrupedStepList().size(); i++)
      {
         QuadrupedTimedStepMessage footstep = message.getQuadrupedStepList().get(i);
         Color regionColor = colors.get(RobotQuadrant.fromByte(footstep.getQuadrupedStepMessage().getRobotQuadrant()));

         footPosition.set(footstep.getQuadrupedStepMessage().getGoalPosition());
         footPosition.addZ(zOffset);

         meshBuilder.addSphere(RADIUS, footPosition, regionColor);
      }

      generateAndQueueForJavaFXUpdate();
   }

   public void clear()
   {
      meshBuilder.clear();
      generateAndQueueForJavaFXUpdate();
   }

   private void generateAndQueueForJavaFXUpdate()
   {
      Mesh mesh = meshBuilder.generateMesh();
      Material material = meshBuilder.generateMaterial();

      synchronized (this)
      {
         this.mesh = mesh;
         this.material = material;
      }
   }

   private void handle(long now)
   {
      synchronized (this)
      {
         meshView.setMesh(mesh);
         meshView.setMaterial(material);
      }
   }

   public void stop()
   {
      executorService.shutdown();
      animationTimer.stop();
   }
}
