package us.ihmc.sensorProcessing.pointClouds.shape;

import georegression.struct.point.Point3D_F64;

import java.util.List;
import java.util.Random;

import us.ihmc.graphics3DAdapter.jme.util.JMEGeometryUtils;
import us.ihmc.sensorProcessing.pointClouds.octree.OccupancyCell;
import us.ihmc.sensorProcessing.pointClouds.octree.OctreeOccupancyExample;
import us.ihmc.sensorProcessing.pointClouds.octree.OctreeOccupancyMap;
import bubo.ptcloud.Octree;

import com.jme3.app.SimpleApplication;
import com.jme3.material.Material;
import com.jme3.material.RenderState.BlendMode;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue.Bucket;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Box;
import com.jme3.system.AppSettings;

public class DisplayPointCloudFileApp extends SimpleApplication
{
   Random rand = new Random(234);

   public String fileName;

   public static void main(String[] args)
   {
      DisplayPointCloudFileApp test1 = new DisplayPointCloudFileApp("data/kinectcloud.txt");
      AppSettings appSettings = new AppSettings(true);
      appSettings.setResolution(1600, 900);
      test1.setSettings(appSettings);
      test1.setShowSettings(false);
      test1.start();
   }

   public DisplayPointCloudFileApp(String fileName)
   {
      this.fileName = fileName;
   }

   @Override
   public void simpleInitApp()
   {
      List<Point3D_F64> cloud = PointCloudTools.readPointCloud(fileName, -1);

      System.out.println("total points: " + cloud.size());
      OctreeOccupancyMap map = OctreeOccupancyExample.mainMethod(cloud);

      Node zUpNode = new Node();
      zUpNode.setLocalRotation(JMEGeometryUtils.getRotationFromJMEToZupCoordinates());

      try
      {
         rootNode.attachChild(zUpNode);
         generateMap(zUpNode, map);
      }
      catch (Exception e)
      {
         this.handleError(e.getMessage(), e);
      }

      cam.setFrustumPerspective(45.0f, ((float) cam.getWidth()) / ((float) cam.getHeight()), 0.05f, 100.0f);
      cam.setLocation(new Vector3f(0, 0, -5));
      cam.lookAtDirection(Vector3f.UNIT_Z, Vector3f.UNIT_Y);
      cam.update();
      flyCam.setMoveSpeed(10);
      flyCam.setDragToRotate(true);
      flyCam.setZoomSpeed(10);
   }

   private void generateMap(Node zUpNode, OctreeOccupancyMap map)
   {
      int i = 0;
      for (Octree o : map.getLeafs())
      {
         Point3D_F64 p0 = o.space.p0;
         Point3D_F64 p1 = o.space.p1;
         p0 = p0.plus(p1);
         Box box = new Box(new Vector3f((float) p0.z / 2, (float) -p0.x / 2, (float) -p0.y / 2), (float) 0.025, (float) 0.025, (float) 0.025);
         Geometry cube = new Geometry("cell" + i, box);
         Material mat1 = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
         mat1.getAdditionalRenderState().setBlendMode(BlendMode.Alpha);
         cube.setQueueBucket(Bucket.Transparent);
         OccupancyCell c = o.getUserData();
         float p = (float) c.getProbability();
         ColorRGBA color = new ColorRGBA(1.0f - p, p, 0f, p);
         mat1.setColor("Color", color);
         cube.setMaterial(mat1);
         zUpNode.attachChild(cube);
         i++;
      }
   }

}
