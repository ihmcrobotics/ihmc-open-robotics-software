package us.ihmc.graphics3DAdapter.jme.lidar;

import us.ihmc.graphics3DAdapter.GPULidar;
import us.ihmc.graphics3DAdapter.GPULidarScanBuffer;
import us.ihmc.graphics3DAdapter.Graphics3DFrameListener;
import us.ihmc.graphics3DAdapter.Graphics3DWorld;
import us.ihmc.graphics3DAdapter.jme.JMEGraphics3DAdapter;
import us.ihmc.graphics3DAdapter.utils.lidar.Graphics3DLidarScan;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.lidar.LidarScan;
import us.ihmc.robotics.geometry.shapes.Sphere3d;

import com.jme3.material.Material;
import com.jme3.material.RenderState.BlendMode;
import com.jme3.math.ColorRGBA;
import com.jme3.renderer.queue.RenderQueue.Bucket;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Sphere;

public class JMEGPULidarTestEnviroment implements Graphics3DFrameListener
{
   private LidarTestParameters params;
   private Graphics3DWorld world;
   private Graphics3DNode lidarNode;
   private Graphics3DNode sphereNode;
   private Graphics3DNode wallNode;
   private Node jmeSphereNode;
   private Graphics3DLidarScan gpuLidarVisualization;
   private Graphics3DLidarScan traceLidarVisualization;
   private GPULidar gpuLidar;
   private RayTracingLidar rayTracingLidar;
   private GPULidarScanBuffer gpuLidarScanBuffer;
   private LidarTestListener testListener;
   private LidarScan gpuScan;
   private LidarScan traceScan;

   public void testAutomatically(LidarTestParameters params, LidarTestListener testListener)
   {
      this.params = params;
      this.testListener = testListener;

      createWorld();
      testWithoutGui();
      startGpuLidar();
   }

   public void testManually(LidarTestParameters params, LidarTestListener testListener)
   {
      this.params = params;
      this.testListener = testListener;

      createWorld();
      testWithGui();
      startGpuLidar();
   }

   private void createWorld()
   {
      world = new Graphics3DWorld("LidarTest", new JMEGraphics3DAdapter());

      sphereNode = new Graphics3DNode("sphere", new Graphics3DObject(new Sphere3d(5.0), YoAppearance.Glass()));
      // sphereNode.getGraphics3DObject().addCoordinateSystem(3);

      wallNode = new Graphics3DNode("wall", new Graphics3DObject());
      wallNode.getGraphics3DObject().addCube(0.01, 10, 10, YoAppearance.Glass());
      wallNode.rotate(Math.PI / 8, Axis.Z);
      wallNode.translate(5, 0, -5);

      Geometry geometry = new Geometry("jmeSphere" + "Geo", new Sphere(200, 200, 5.0f, false, true));
      Material material = new Material(((JMEGraphics3DAdapter) world.getGraphics3DAdapter()).getRenderer().getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
      material.setColor("Color", new ColorRGBA(0, 1, 0, 0.5f));
      material.getAdditionalRenderState().setBlendMode(BlendMode.Alpha);
      geometry.setMaterial(material);
      geometry.setQueueBucket(Bucket.Transparent);
      jmeSphereNode = new Node("jmeSphereNode");
      jmeSphereNode.move(1.5f, 2.5f, -0.5f);
      jmeSphereNode.attachChild(geometry);
      ((JMEGraphics3DAdapter) world.getGraphics3DAdapter()).getRenderer().getZUpNode().attachChild(jmeSphereNode);

      lidarNode = new Graphics3DNode("lidar", new Graphics3DObject());
      lidarNode.getGraphics3DObject().addModelFile("models/hokuyo.dae", YoAppearance.Black());

      gpuLidarVisualization = new Graphics3DLidarScan(world, "gpuLidar", params.getScansPerSweep(), params.getMinRange(), params.getMaxRange(),
              params.getShowScanRays(), params.getShowGpuPoints(), YoAppearance.Red());
      traceLidarVisualization = new Graphics3DLidarScan(world, "traceLidar", params.getScansPerSweep(), params.getMinRange(), params.getMaxRange(), false,
              params.getShowTracePoints(), YoAppearance.Blue());

      rayTracingLidar = new RayTracingLidar(world, params.getScansPerSweep(), params.getLidarSweepEndAngle() - params.getLidarSweepStartAngle(),
              params.getMinRange(), params.getMaxRange(), 0);
//    rayTracingLidar.addCollisionNodes(sphereNode.getName());
      rayTracingLidar.addCollisionNodes(jmeSphereNode.getName());
      rayTracingLidar.addCollisionNodes(wallNode.getName());

      world.addChild(lidarNode);
      world.addChild(wallNode);
//    world.addChild(sphereNode);
   }

   private void testWithGui()
   {
      world.startWithGui(10, -20, 20, 1800, 1080);
      world.addFrameListener(this);
      world.fixCameraOnNode(sphereNode);
   }

   private void testWithoutGui()
   {
      world.startWithoutGui();
      world.addFrameListener(this);
   }

   private void startGpuLidar()
   {
      gpuLidar = world.getGraphics3DAdapter().createGPULidar(params.getLidarScanParameters());
      gpuLidarScanBuffer = new GPULidarScanBuffer(params.getLidarScanParameters());
      gpuLidar.addGPULidarListener(gpuLidarScanBuffer);
      gpuLidar.setTransformFromWorld(lidarNode.getTransform(), 0.0);
   }

   public void postFrame(double timePerFrame)
   {
      if (gpuLidarScanBuffer == null)
         return;
      
      while (!gpuLidarScanBuffer.isEmpty())
      {
         gpuScan = gpuLidarScanBuffer.poll();
         
//         System.out.println("Rotation = " + gpuScan.getStartTransform());
         
         gpuLidarVisualization.update(gpuScan);

         traceScan = rayTracingLidar.scan(gpuScan.getStartTransform());
         traceLidarVisualization.update(traceScan);
      }

      params.rotate(timePerFrame);

      if (params.testIsOver())
      {
         testListener.stop();
      }

      lidarNode.rotate(params.getRotationSpeed() * timePerFrame, Axis.X);
      gpuLidar.setTransformFromWorld(lidarNode.getTransform(), 0.0);

      if ((gpuScan != null) && (traceScan != null) && (testListener != null))
      {
         testListener.notify(gpuScan, traceScan);
      }
   }

   public Graphics3DWorld getWorld()
   {
      return world;
   }
}
