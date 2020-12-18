package us.ihmc.ihmcPerception.lineSegmentDetector;

import com.jme3.app.SimpleApplication;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.*;
import com.jme3.scene.control.CameraControl;
import com.jme3.scene.shape.Box;
import com.jme3.system.AppSettings;
import com.jme3.util.BufferUtils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import sensor_msgs.PointCloud2;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

import java.awt.*;
import java.net.URI;
import java.net.URISyntaxException;

public class JMEPointCloudViewer extends SimpleApplication
{
   private CameraNode camNode;

   private Node camGrandNode;
   private Node camParentNode;
   private Geometry camBoxGeom;

   private boolean camNavEnabled;
   private boolean camNavShiftEnabled;
   private boolean camNavLiftEnabled;
   public boolean firstSaved = false;

   private RealSenseL515PointCloudROS1Subscriber pointCloudSubscriber;
   private Vector3f[] pointVectors3f;
   public Color[] pointColors;
   public Point3D[] points3D;
   private Mesh pcl1;
   private Geometry g1;

   RosMainNode rosMainNode;
   private Mesh lineMesh;

   int ind = 0;

   public JMEPointCloudViewer() throws URISyntaxException
   {
      super();

      // setupROS1PointCloudReceiver();
      //      setupRealSenseDepthImageReceiver();
   }

   public void setupROS1PointCloudReceiver() throws URISyntaxException
   {
      rosMainNode = new RosMainNode(new URI("http://localhost:11311"), "RealSenseL515DataViewer", true);
      this.pointCloudSubscriber = new RealSenseL515PointCloudROS1Subscriber(rosMainNode)
      {
         @Override
         protected void cloudReceived(PointCloud2 cloud)
         {
            System.out.println("Received Points:" + cloud.getData().array().length);
            RosPointCloudSubscriber.UnpackedPointCloud unpackedPointCloud = RosPointCloudSubscriber.unpackPointsAndIntensities(cloud);

            int total = unpackedPointCloud.getPoints().length;
            points3D = unpackedPointCloud.getPoints();
            pointColors = unpackedPointCloud.getPointColors();

            pointVectors3f = new Vector3f[total];
            copyPointsEuclidToJME(points3D, pointVectors3f);
         }
      };
      rosMainNode.execute();
   }

   public void copyPointsEuclidToJME(Point3D[] pclEuclid, Vector3f[] pclJME)
   {
      for (int i = 0; i < pclEuclid.length; i++)
      {
         // System.out.println(pclEuclid.length + " " + pclJME.length + " " + i);
         pclJME[i] = new Vector3f(pclEuclid[i].getX32(), pclEuclid[i].getY32(), pclEuclid[i].getZ32());
      }
   }

   public static void main(String[] args) throws URISyntaxException
   {

      System.loadLibrary("opencv_java440"); // TODO: Change based on system agnostic parameter [System.loadLibrary(Core.NATIVE_LIBRARY_NAME);]

      Mat m = new Mat(4, 10, CvType.CV_8UC1, new Scalar(0));
      // System.out.println("Hello OpenCV" + m.dump());

      JMEPointCloudViewer app = new JMEPointCloudViewer();
      app.setShowSettings(false);
      AppSettings settings = new AppSettings(true);
      settings.put("Width", 2400);
      settings.put("Height", 1600);
      settings.put("Title", "OctoBase Java Monkey Engine 3.2");
      settings.put("VSync", false);
      settings.put("Samples", 4);
      app.setSettings(settings);

      app.start();
   }

   public Geometry generatePointCloudMesh(Mesh pcl, ColorRGBA color)
   {

      pcl.setMode(Mesh.Mode.Points);
      pcl.setStatic();
      pcl.updateBound();

      Material unshadedMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
      unshadedMat.setColor("Color", color);
      unshadedMat.setFloat("PointSize", 3f);
      unshadedMat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Off);

      Material litMat = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
      litMat.setColor("Ambient", color);
      litMat.setColor("Diffuse", color);
      litMat.setColor("Specular", color);
      litMat.setBoolean("UseMaterialColors", true);
      litMat.setFloat("Shininess", 10f);

      Material mat = unshadedMat;

      Geometry g = new Geometry("Point Cloud", pcl);
      g.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
      g.setQueueBucket(RenderQueue.Bucket.Opaque);
      g.setMaterial(mat);
      g.updateModelBound();

      return g;
   }

   @Override
   public void simpleInitApp()
   {
      flyCam.setEnabled(false);
      camNavEnabled = false;
      camNavShiftEnabled = false;

      pcl1 = new Mesh();
      g1 = generatePointCloudMesh(pcl1, ColorRGBA.Yellow);

      Material lineMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
      lineMat.setColor("Color", ColorRGBA.Red);
      lineMesh = new Mesh();
      lineMesh.setMode(Mesh.Mode.Lines);
      Geometry gLine = new Geometry("Line", lineMesh);
      gLine.setMaterial(lineMat);
      rootNode.attachChild(gLine);

      rootNode.attachChild(g1);
      rootNode.updateModelBound();

      camGrandNode = new Node();
      camParentNode = new Node();
      camGrandNode.attachChild(camParentNode);
      camNode = new CameraNode("CamNode", cam);
      camParentNode.attachChild(camNode);
      camNode.setControlDir(CameraControl.ControlDirection.SpatialToCamera);
      camNode.setLocalTranslation(0, 0, -5);

      Box b = new Box(0.01f, 0.01f, 0.01f);
      camBoxGeom = new Geometry("Box", b);
      Material voxMat = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
      voxMat.setColor("Ambient", ColorRGBA.White);
      voxMat.setColor("Diffuse", ColorRGBA.White);
      voxMat.setColor("Specular", ColorRGBA.White);
      voxMat.setBoolean("UseMaterialColors", true);
      voxMat.setFloat("Shininess", 2);
      camBoxGeom.setMaterial(voxMat);
      camBoxGeom.setLocalTranslation(0, 0, 0);
      rootNode.attachChild(camBoxGeom);

      DirectionalLight sun1 = new DirectionalLight();
      sun1.setDirection(new Vector3f(-30f, -10f, 20f).normalizeLocal());
      sun1.setColor(ColorRGBA.White);
      rootNode.addLight(sun1);
      DirectionalLight sun2 = new DirectionalLight();
      sun2.setDirection(new Vector3f(10f, 10f, -20f).normalizeLocal());
      sun2.setColor(ColorRGBA.White);
      rootNode.addLight(sun2);

      rootNode.attachChild(camGrandNode);
      JMEViewerKeyBindings keyBinder = new JMEViewerKeyBindings();
      keyBinder.setNodes(camParentNode, camGrandNode, camNode, camBoxGeom);
      keyBinder.setKeyBindings(inputManager);
      keyBinder.setViewer(this);
   }

   @Override
   public void simpleUpdate(float tpf)
   {
      super.simpleUpdate(tpf);

      if (pointVectors3f != null)
      {
         pcl1.setBuffer(VertexBuffer.Type.Position, 3, BufferUtils.createFloatBuffer(pointVectors3f));
         pcl1.updateBound();
      }
   }
}