package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImFloat;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.gdx.ui.graphics.GDXBoxRenderer;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import lidar_obstacle_detection.GDXBoxesMessage;

public class GDXROS1BoxVisualizer implements RenderableProvider
{
   private static final int MAX_POINTS = 50000;

   private final RosMainNode ros1Node;
   private final String ros1BoxTopic;
   private final ReferenceFrame sensorBaseFrame;
   private final RigidBodyTransformReadOnly baseToSensorTransform;
   private final RigidBodyTransform tempBaseToSensorTransform = new RigidBodyTransform();
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   private final ImFloat tuneX = new ImFloat(0.275f);
   private final ImFloat tuneY = new ImFloat(0.052f);
   private final ImFloat tuneZ = new ImFloat(0.14f);
   private final ImFloat tuneYaw = new ImFloat(0.01f);
   private final ImFloat tunePitch = new ImFloat(24.0f);
   private final ImFloat tuneRoll = new ImFloat(-0.045f);

   private boolean packingA = true;
//   private final RecyclingArrayList<Point3D32> pointsA = new RecyclingArrayList<>(MAX_POINTS, Point3D32::new);
//   private final RecyclingArrayList<Point3D32> pointsB = new RecyclingArrayList<>(MAX_POINTS, Point3D32::new);

   private final ResettableExceptionHandlingExecutorService threadQueue;

   private GDXBoxRenderer BoxRenderer = new GDXBoxRenderer();
//   private final RecyclingArrayList<Point3D32> pointsToRender = new RecyclingArrayList<>(Point3D32::new);

   private boolean enabled = false;

   private long receivedCount = 0;
   private final ImGuiPlot receivedPlot = new ImGuiPlot("", 1000, 230, 20);

   public GDXROS1BoxVisualizer(RosMainNode ros1Node,
                                      String ros1BoxTopic,
                                      ReferenceFrame sensorBaseFrame,
                                      RigidBodyTransformReadOnly baseToSensorTransform)
   {
      this.ros1Node = ros1Node;
      this.ros1BoxTopic = ros1BoxTopic;
      this.sensorBaseFrame = sensorBaseFrame;
      this.baseToSensorTransform = baseToSensorTransform;

      threadQueue = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

      ros1Node.attachSubscriber(ros1BoxTopic, new AbstractRosTopicSubscriber<GDXBoxesMessage>(GDXBoxesMessage._TYPE)
      {
         @Override
         public void onNewMessage(GDXBoxesMessage Boxes)
         {
            ++receivedCount;
            queueRenderBoxes(Boxes);
         }

      });

   }

   private void queueRenderBoxes(GDXBoxesMessage boxes)
   {
      ++receivedCount;
      if (enabled)
      {
         threadQueue.clearQueueAndExecute(() ->
         {
            try
            {
//               System.out.printf(boxes.getBoundingBoxes()[0]);

               //               boolean hasColors = true;
//               PointCloudData pointCloudData = new PointCloudData(message, MAX_POINTS, hasColors);
//
//               pointCloudData.flipToZUp();
//
//               // Should be tuned somewhere else
//               //               baseToSensorTransform.setToZero();
//               //               baseToSensorTransform.appendTranslation(tuneX.get(), tuneY.get(), tuneZ.get());
//               //               double pitch = Math.toRadians(90.0 - tunePitch.get());
//               //               baseToSensorTransform.appendOrientation(new YawPitchRoll(tuneYaw.get(), pitch, tuneRoll.get()));
//
//               sensorBaseFrame.getTransformToDesiredFrame(transformToWorld, ReferenceFrame.getWorldFrame());
//               transformToWorld.multiply(baseToSensorTransform);
//               pointCloudData.applyTransform(transformToWorld);
//
//               synchronized (pointsToRender)
//               {
//                  RecyclingArrayList<Point3D32> pointsToPack = packingA ? pointsA : pointsB;
//                  pointsToPack.clear();
//                  for (int i = 0; i < pointCloudData.getNumberOfPoints() && packingA; i++)
//                  {
//                     pointsToPack.add().set(pointCloudData.getPointCloud()[i]);
//                  }
//                  packingA = !packingA;
//               }
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
      BoxRenderer.create(MAX_POINTS);
   }

   public void updateMesh()
   {
      if (enabled)
      {
//         BoxRenderer.clear();
//         synchronized (pointsToRender)
//         {
//            RecyclingArrayList<Point3D32> pointsToRead = packingA ? pointsB : pointsA;
//            for (Point3D32 point : pointsToRead)
//            {
//               pointsToRender.add().set(point);
//            }
//         }
//
//         pointCloudRenderer.setPointsToRender(pointsToRender);
//         if (!pointsToRender.isEmpty())
//         {
//            pointCloudRenderer.updateMesh();
//         }
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text(ros1BoxTopic);
      receivedPlot.render(receivedCount);

      // 0.25, 0.0, 0.11
      ImGui.dragFloat("TuneX", tuneX.getData(), 0.0001f, 0.21f, 0.32f);
      ImGui.dragFloat("TuneY", tuneY.getData(), 0.0001f, 0.01f, 0.07f);
      ImGui.dragFloat("TuneZ", tuneZ.getData(), 0.0001f, 0.12f, 0.16f);
      ImGui.dragFloat("TuneYaw", tuneYaw.getData(), 0.0001f, 0.01f, 0.08f);
      ImGui.dragFloat("TunePitch", tunePitch.getData(), 0.0001f, 20.0f, 30.0f);
      ImGui.dragFloat("TuneRoll", tuneRoll.getData(), 0.0001f, -5.0f, 5.0f);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (enabled)
         BoxRenderer.getRenderables(renderables, pool);
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }
}
