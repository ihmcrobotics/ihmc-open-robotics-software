package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import org.jboss.netty.buffer.ChannelBuffer;
import sensor_msgs.CameraInfo;
import sensor_msgs.Image;
import sensor_msgs.PointCloud2;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.gdx.BufferBasedColorProvider;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXROS1Visualizer;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class GDXROS1PointCloudVisualizer extends ImGuiGDXROS1Visualizer implements RenderableProvider
{
   private static final int MAX_POINTS = 100000;

   private final String ros1PointCloudTopic;
   private ReferenceFrame frame;
   private RigidBodyTransformReadOnly transformAfterFrame;
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

//   private final ImFloat tuneX = new ImFloat(0.275f);
//   private final ImFloat tuneY = new ImFloat(0.052f);
//   private final ImFloat tuneZ = new ImFloat(0.14f);
//   private final ImFloat tuneYaw = new ImFloat(0.01f);
//   private final ImFloat tunePitch = new ImFloat(24.0f);
//   private final ImFloat tuneRoll = new ImFloat(-0.045f);
   private final ImGuiPlot receivedPlot = new ImGuiPlot("", 1000, 230, 20);

   private boolean packingA = true;
   private final RecyclingArrayList<Point3D32> pointsA = new RecyclingArrayList<>(MAX_POINTS, Point3D32::new);
   private final RecyclingArrayList<Point3D32> pointsB = new RecyclingArrayList<>(MAX_POINTS, Point3D32::new);

   private final ResettableExceptionHandlingExecutorService threadQueue;

   private final GDXPointCloudRenderer pointCloudRenderer = new GDXPointCloudRenderer();
   private final RecyclingArrayList<Point3D32> pointsToRender = new RecyclingArrayList<>(Point3D32::new);

   private AbstractRosTopicSubscriber<PointCloud2> subscriber;
   private long receivedCount = 0;

   //Camera color stuff
   private Color color = Color.WHITE;
   private String rosVideoTopic = null;
   private String rosCameraInfoTopic = null;

   private AbstractRosTopicSubscriber<Image> imageSubscriber = null;
   private AbstractRosTopicSubscriber<CameraInfo> cameraInfoSubscriber = null;

   private volatile Image image;
   private volatile CameraInfo cameraInfo;

   private GDXPointCloudRenderer.ColorProvider colorProvider = null;

   private boolean flipToZUp = false;

   public GDXROS1PointCloudVisualizer(String title, String ros1PointCloudTopic)
   {
      super(title);
      this.ros1PointCloudTopic = ros1PointCloudTopic;
      threadQueue = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   }

   @Override
   public void create()
   {
      super.create();
      pointCloudRenderer.create(MAX_POINTS);
   }

   public void setFrame(ReferenceFrame frame)
   {
      this.frame = frame;
   }

   public void setFrame(ReferenceFrame frame, RigidBodyTransformReadOnly transformAfterFrame)
   {
      this.frame = frame;
      this.transformAfterFrame = transformAfterFrame;
   }

   @Override
   public void subscribe(RosNodeInterface ros1Node)
   {
      subscriber = new AbstractRosTopicSubscriber<PointCloud2>(PointCloud2._TYPE)
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud2)
         {
            ++receivedCount;
            queueRenderPointCloud(pointCloud2);
         }
      };
      ros1Node.attachSubscriber(ros1PointCloudTopic, subscriber);

      if (rosVideoTopic != null) {
         imageSubscriber = new AbstractRosTopicSubscriber<Image>(sensor_msgs.Image._TYPE)
         {
            @Override
            public void onNewMessage(Image image)
            {
               GDXROS1PointCloudVisualizer.this.image = image;
            }
         };
         ros1Node.attachSubscriber(rosVideoTopic, imageSubscriber);

         cameraInfoSubscriber = new AbstractRosTopicSubscriber<CameraInfo>(sensor_msgs.CameraInfo._TYPE)
         {
            @Override
            public void onNewMessage(CameraInfo info)
            {
               GDXROS1PointCloudVisualizer.this.cameraInfo = info;
            }
         };
         ros1Node.attachSubscriber(rosCameraInfoTopic, cameraInfoSubscriber);
      }
   }

   @Override
   public void unsubscribe(RosNodeInterface ros1Node)
   {
      ros1Node.removeSubscriber(subscriber);
      if (imageSubscriber != null) {
         ros1Node.removeSubscriber(imageSubscriber);
         imageSubscriber = null;
      }
   }

   private void queueRenderPointCloud(PointCloud2 message)
   {
      if (isActive())
      {
         threadQueue.clearQueueAndExecute(() ->
         {
            try
            {
               boolean hasColors = false;
               PointCloudData pointCloudData = new PointCloudData(message, MAX_POINTS, hasColors);

               if (imageSubscriber != null) {
                  PointCloudData pointCloudDataForColors = new PointCloudData(message, MAX_POINTS, hasColors);

                  Image image = this.image;
                  CameraInfo info = this.cameraInfo;

                  //Applies the following transform:
                  //     [fx'  0  cx' Tx]
                  // P = [ 0  fy' cy' Ty]
                  //     [ 0   0   1   0]
                  //This transforms a 3D point to a 2D pixel coordinate
                  //https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html#:~:text=%23%C2%A0Projection/camera%C2%A0matrix,a%C2%A0stereo%C2%A0pair%2E
                  pointCloudDataForColors.applyTransform(new RigidBodyTransform(info.getP()));

                  ChannelBuffer data = image.getData();
                  int zeroedIndex = 0;

                  Pixmap pixmap = new Pixmap(image.getWidth(), image.getHeight(), Pixmap.Format.RGBA8888);
                  for (int y = 0; y < image.getHeight(); y++)
                  {
                     for (int x = 0; x < image.getWidth(); x++)
                     {
                        int r = Byte.toUnsignedInt(data.getByte(zeroedIndex + 0));
                        int g = Byte.toUnsignedInt(data.getByte(zeroedIndex + 1));
                        int b = Byte.toUnsignedInt(data.getByte(zeroedIndex + 2));
                        int a = 255;
                        zeroedIndex += 3;
                        int rgb8888 = (r << 24) | (g << 16) | (b << 8) | a;
                        pixmap.drawPixel(x, y, rgb8888);
                     }
                  }

                  BufferBasedColorProvider provider = new BufferBasedColorProvider();

                  for (Point3D point3D : pointCloudDataForColors.getPointCloud()) {
                     Point2D point = new Point2D(point3D.getX(), point3D.getY());
                     provider.add(new Color(pixmap.getPixel((int) point.getX(), (int) point.getY())));
                  }

                  this.colorProvider = provider;
               }

               if (flipToZUp)
                  pointCloudData.flipToZUp();

               // Should be tuned somewhere else
//               baseToSensorTransform.setToZero();
//               baseToSensorTransform.appendTranslation(tuneX.get(), tuneY.get(), tuneZ.get());
//               double pitch = Math.toRadians(90.0 - tunePitch.get());
//               baseToSensorTransform.appendOrientation(new YawPitchRoll(tuneYaw.get(), pitch, tuneRoll.get()));

               if (frame != null)
               {
                  frame.getTransformToDesiredFrame(transformToWorld, ReferenceFrame.getWorldFrame());
                  if (transformAfterFrame != null)
                     transformToWorld.multiply(transformAfterFrame);
                  pointCloudData.applyTransform(transformToWorld);
               }

               synchronized (pointsToRender)
               {
                  RecyclingArrayList<Point3D32> pointsToPack = packingA ? pointsA : pointsB;
                  pointsToPack.clear();
                  for (int i = 0; i < pointCloudData.getNumberOfPoints() && packingA; i++)
                  {
                     pointsToPack.add().set(pointCloudData.getPointCloud()[i]);
                  }
                  packingA = !packingA;
               }
            }
            catch (Exception e)
            {
               LogTools.error(e.getMessage());
               e.printStackTrace();
            }
         });
      }
   }

   @Override
   public void update()
   {
      super.update();
      updateMesh();
   }

   public void updateMesh()
   {
      updateMesh(0.0f);
   }

   public void updateMesh(float alpha)
   {
      if (isActive())
      {
         pointsToRender.clear();
         synchronized (pointsToRender)
         {
            RecyclingArrayList<Point3D32> pointsToRead = packingA ? pointsB : pointsA;
            for (Point3D32 point : pointsToRead)
            {
               pointsToRender.add().set(point);
            }
         }

         if (colorProvider != null) {
            pointCloudRenderer.setPointsToRender(pointsToRender, colorProvider);
         }
         else
         {
            pointCloudRenderer.setPointsToRender(pointsToRender, color);
         }

         if (!pointsToRender.isEmpty())
         {
            pointCloudRenderer.updateMesh();
         }
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();

      ImGui.text(ros1PointCloudTopic);
      receivedPlot.render(receivedCount);

      // 0.25, 0.0, 0.11
//      ImGui.dragFloat("TuneX", tuneX.getData(), 0.0001f, 0.21f, 0.32f);
//      ImGui.dragFloat("TuneY", tuneY.getData(), 0.0001f, 0.01f, 0.07f);
//      ImGui.dragFloat("TuneZ", tuneZ.getData(), 0.0001f, 0.12f, 0.16f);
//      ImGui.dragFloat("TuneYaw", tuneYaw.getData(), 0.0001f, 0.01f, 0.08f);
//      ImGui.dragFloat("TunePitch", tunePitch.getData(), 0.0001f, 20.0f, 30.0f);
//      ImGui.dragFloat("TuneRoll", tuneRoll.getData(), 0.0001f, -5.0f, 5.0f);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (isActive())
         pointCloudRenderer.getRenderables(renderables, pool);
   }

   public void setFlipToZUp(boolean flipToZUp)
   {
      this.flipToZUp = flipToZUp;
   }

   /**
    * Overrides {@link #setColor(Color)} - will get color from camera. Should be called BEFORE subscribing
    */
   public void setROSVideoTopic(String videoTopic, String cameraInfoTopic) {
      this.color = null;
      this.rosVideoTopic = videoTopic;
      this.rosCameraInfoTopic = cameraInfoTopic;
   }

   /**
    * Overrides {@link #setROSVideoTopic(String, String)} - will always use passed color (not from camera)
    */
   public void setColor(Color color)
   {
      this.color = color;
      this.rosVideoTopic = null;
   }
}
