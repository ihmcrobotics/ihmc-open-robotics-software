package us.ihmc.robotEnvironmentAwareness.polygonizer;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import javafx.animation.AnimationTimer;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.geometry.REAGraphics3DTools;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder;

public class MultiplePointCloudViewer extends AnimationTimer
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme(MultiplePointCloudViewer.class.getSimpleName()));

   private static final TypedTopicTheme<Boolean> View = apiFactory.createTypedTopicTheme("View");
   private static final TypedTopicTheme<List<Input>> InputTheme = apiFactory.createTypedTopicTheme("Input");

   public static final Topic<List<Input>> PointCloudInput = Root.topic(InputTheme);
   public static final Topic<Boolean> ViewPointCloud = Root.topic(View);

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private final ExecutorService executorService;

   private final BooleanProperty showPointCloud = new SimpleBooleanProperty(this, "showPointCloud", true);
   private final DoubleProperty pointCloudSize = new SimpleDoubleProperty(this, "pointCloudSize", 0.0025);

   private final Group rootNode = new Group();
   private final List<PointCloudViewer> pointCloudViewersRendered = new ArrayList<>();
   private final AtomicReference<List<PointCloudViewer>> pointCloudViewersToRender = new AtomicReference<>(null);

   public MultiplePointCloudViewer(JavaFXMessager messager, ExecutorService executorService)
   {
      this.executorService = executorService;
      messager.addTopicListener(PointCloudInput, this::processInPutOnThread);
   }

   private void processInPutOnThread(Collection<Input> inputs)
   {
      executorService.execute(() -> submit(inputs));
   }

   @Override
   public void handle(long now)
   {
      List<PointCloudViewer> newViewers = pointCloudViewersToRender.getAndSet(null);

      if (newViewers != null)
      {
         rootNode.getChildren().clear();
         pointCloudViewersRendered.clear();
         pointCloudViewersRendered.addAll(newViewers);
         pointCloudViewersRendered.forEach(viewer -> rootNode.getChildren().add(viewer.meshViewRendered));
      }

      pointCloudViewersRendered.forEach(viewer -> viewer.render());
   }

   public void submit(Collection<Input> inputs)
   {
      List<PointCloudViewer> newViewers = new ArrayList<>();

      for (Input input : inputs)
      {
         PointCloudViewer newViewer = new PointCloudViewer();
         newViewer.submitForRendering(REAGraphics3DTools.pointcloud(input.pointCloud, input.color, pointCloudSize.get()));
         newViewers.add(newViewer);
      }

      pointCloudViewersToRender.set(newViewers);
   }

   public Node getRootNode()
   {
      return rootNode;
   }

   public static List<Input> toInputList(Collection<PlanarRegionSegmentationRawData> data)
   {
      return data.stream().map(MultiplePointCloudViewer::toInput).collect(Collectors.toList());
   }

   public static Input toInput(PlanarRegionSegmentationRawData data)
   {
      return new Input(data.getRegionId(), OcTreeMeshBuilder.getRegionColor(data.getRegionId()), data.getPointCloudInWorld());
   }

   private class PointCloudViewer
   {
      private final BooleanProperty viewGraphic = new SimpleBooleanProperty(this, "viewPointCloud", true);
      private final MeshView meshViewRendered = new MeshView();
      private final AtomicReference<MeshView> meshViewToRenderRef = new AtomicReference<>(null);

      public PointCloudViewer()
      {
         setMouseTransparent(true);
      }

      void render()
      {
         if (!viewGraphic.get() || !showPointCloud.get())
         {
            meshViewRendered.setMesh(null);
            meshViewRendered.setMaterial(null);
            return;
         }

         MeshView meshViewToRender = meshViewToRenderRef.get();
         if (meshViewToRender != null)
         {
            meshViewRendered.setMesh(meshViewToRender.getMesh());
            meshViewRendered.setMaterial(meshViewToRender.getMaterial());
         }
      }

      void submitForRendering(MeshView newMeshViewToRender)
      {
         meshViewToRenderRef.set(newMeshViewToRender);
      }

      void setMouseTransparent(boolean value)
      {
         meshViewRendered.setMouseTransparent(value);
      }
   }

   public static class Input
   {
      private final int id;
      private final Color color;
      private final List<Point3D> pointCloud;

      public Input(int id, Color color, List<Point3D> pointCloud)
      {
         this.id = id;
         this.color = color;
         this.pointCloud = pointCloud;
      }

      public int getId()
      {
         return id;
      }

      public Color getColor()
      {
         return color;
      }

      public List<Point3D> getPointCloud()
      {
         return pointCloud;
      }
   }
}
