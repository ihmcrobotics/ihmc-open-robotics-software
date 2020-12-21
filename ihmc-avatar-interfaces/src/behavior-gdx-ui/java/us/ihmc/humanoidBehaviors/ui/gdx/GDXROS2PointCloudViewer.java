package us.ihmc.humanoidBehaviors.ui.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.particles.ParticleSystem;
import com.badlogic.gdx.graphics.g3d.particles.batches.PointSpriteParticleBatch;
import com.badlogic.gdx.graphics.g3d.particles.renderers.PointSpriteControllerRenderData;
import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import org.lwjgl.opengl.GL30;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.gdx.GDX3DApplication;
import us.ihmc.gdx.GDXApplicationCreator;
import us.ihmc.gdx.GDXModelPrimitives;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.SingleThreadSizeOneQueueExecutor;

import java.util.concurrent.atomic.AtomicReference;

public class GDXROS2PointCloudViewer extends GDX3DApplication
{
   private final ROS2Node ros2Node;
   private final SingleThreadSizeOneQueueExecutor threadQueue;

   private AtomicReference<Point3D32[]> points = new AtomicReference<>();

   public GDXROS2PointCloudViewer(ROS2Node ros2Node, ROS2Topic<?> topic)
   {
      this.ros2Node = ros2Node;
      threadQueue = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

      if (topic.getType().equals(LidarScanMessage.class))
      {
         new IHMCROS2Callback<>(ros2Node, topic.withType(LidarScanMessage.class), this::queueRenderLidarScan);
      }
      else if (topic.getType().equals(StereoVisionPointCloudMessage.class))
      {
         new IHMCROS2Callback<>(ros2Node, topic.withType(StereoVisionPointCloudMessage.class), this::queueRenderStereoVisionPointCloud);
      }

      Lwjgl3Application lwjgl3App = GDXApplicationCreator.launchGDXApplication(new PrivateGDX3DApplication(), "GDX3DDemo", 1100, 800);
      lwjgl3App.getGraphics().getGL30().
   }

   private void queueRenderStereoVisionPointCloud(StereoVisionPointCloudMessage message)
   {
      threadQueue.submitTask(() ->
      {
         Point3D32[] points = PointCloudCompression.decompressPointCloudToArray32(message);
         int[] colors = PointCloudCompression.decompressColorsToIntArray(message);

         this.points.set(points);
//         buildMesh(points, colors);
      });
   }

   private void queueRenderLidarScan(LidarScanMessage message)
   {
      threadQueue.submitTask(() ->
      {
         int numberOfPoints = message.getScan().size() / 3;
         Point3D32[] points = new Point3D32[numberOfPoints];
         int[] colors = new int[numberOfPoints];

         for (int i = 0; i < numberOfPoints; i++)
         {
            points[i] = new Point3D32();
            points[i].setX(message.getScan().get(i * 3));
            points[i].setY(message.getScan().get(i * 3 + 1));
            points[i].setZ(message.getScan().get(i * 3 + 2));

            colors[i] = 0;
         }

         this.points.set(points);
//         buildMesh(points, colors);
      });
   }

   class PrivateGDX3DApplication extends GDX3DApplication
   {
      private PointSpriteParticleBatch pointSpriteParticleBatch;
      private ParticleSystem particleSystem;

      @Override
      public void create()
      {
         super.create();

         addModelInstance(new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3)));

//         particleSystem = new ParticleSystem();

         pointSpriteParticleBatch = new PointSpriteParticleBatch();
         pointSpriteParticleBatch.setCamera(getCamera3D());
//         particleSystem.add(pointSpriteParticleBatch);
//         pointSpriteParticleBatch.

      }

      @Override
      public void render()
      {
//         Gdx.gl.glEnable(GL20.GL_VERTEX_PROGRAM_POINT_SIZE);
//         Gdx.gl.glEnable(GL30.GL_POINT_SPRITE);

         Point3D32[] pointsToRender = points.get();

         renderBefore();

         pointSpriteParticleBatch.begin();

         PointSpriteControllerRenderData data = new PointSpriteControllerRenderData();
         data.positionChannel.setCapacity(pointsToRender.length);

         for (int i = 0; i < pointsToRender.length; i++)
         {
            data.positionChannel.data[i * 3 + 0] = pointsToRender[i].getX32();
            data.positionChannel.data[i * 3 + 1] = pointsToRender[i].getY32();
            data.positionChannel.data[i * 3 + 2] = pointsToRender[i].getZ32();
         }


         pointSpriteParticleBatch.draw(data);
         pointSpriteParticleBatch.end();
         getModelBatch().render(pointSpriteParticleBatch, getEnvironment());

         renderAfter();
      }
   }

   public static void main(String[] args)
   {
      new GDXROS2PointCloudViewer(ROS2Tools.createInterprocessROS2Node("point_cloud_viewer"), ROS2Tools.MULTISENSE_LIDAR_SCAN);
   }
}
