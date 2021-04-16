package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import sensor_msgs.PointCloud2;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class GDXROS1PointCloudVisualizer implements RenderableProvider
{
   private static final int MAX_POINTS = 50000;

   private final String ros1PointCloudTopic;
   private final ReferenceFrame sensorBaseFrame;
   private final RigidBodyTransformReadOnly baseToSensorTransform;
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

   private GDXPointCloudRenderer pointCloudRenderer = new GDXPointCloudRenderer();
   private final RecyclingArrayList<Point3D32> pointsToRender = new RecyclingArrayList<>(Point3D32::new);

   private boolean enabled = false;
   private AbstractRosTopicSubscriber<PointCloud2> subscriber;
   private long receivedCount = 0;


   public GDXROS1PointCloudVisualizer(String ros1PointCloudTopic,
                                      ReferenceFrame sensorBaseFrame,
                                      RigidBodyTransformReadOnly baseToSensorTransform)
   {
      this.ros1PointCloudTopic = ros1PointCloudTopic;
      this.sensorBaseFrame = sensorBaseFrame;
      this.baseToSensorTransform = baseToSensorTransform;

      threadQueue = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }

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
   }

   public void unsubscribe(RosNodeInterface ros1Node)
   {
      ros1Node.removeSubscriber(subscriber);
   }

   private void queueRenderPointCloud(PointCloud2 message)
   {
      if (enabled)
      {
         threadQueue.clearQueueAndExecute(() ->
         {
            try
            {
               boolean hasColors = true;
               PointCloudData pointCloudData = new PointCloudData(message, MAX_POINTS, hasColors);

               pointCloudData.flipToZUp();

               // Should be tuned somewhere else
//               baseToSensorTransform.setToZero();
//               baseToSensorTransform.appendTranslation(tuneX.get(), tuneY.get(), tuneZ.get());
//               double pitch = Math.toRadians(90.0 - tunePitch.get());
//               baseToSensorTransform.appendOrientation(new YawPitchRoll(tuneYaw.get(), pitch, tuneRoll.get()));

               sensorBaseFrame.getTransformToDesiredFrame(transformToWorld, ReferenceFrame.getWorldFrame());
               transformToWorld.multiply(baseToSensorTransform);
               pointCloudData.applyTransform(transformToWorld);

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

   public void create()
   {
      pointCloudRenderer.create(MAX_POINTS);
   }

   public void updateMesh()
   {
      updateMesh(0.0f);
   }

   public void updateMesh(float alpha)
   {
      if (enabled)
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

         pointCloudRenderer.setPointsToRender(pointsToRender);
         if (!pointsToRender.isEmpty())
         {
            pointCloudRenderer.updateMesh(alpha);
         }
      }
   }

   public void renderImGuiWidgets()
   {
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
      if (enabled)
         pointCloudRenderer.getRenderables(renderables, pool);
   }
}
