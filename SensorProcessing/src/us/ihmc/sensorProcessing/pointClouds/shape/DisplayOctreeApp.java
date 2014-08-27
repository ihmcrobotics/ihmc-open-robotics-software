package us.ihmc.sensorProcessing.pointClouds.shape;

import georegression.struct.point.Point3D_F64;

import java.util.Iterator;
import java.util.List;

import us.ihmc.graphics3DAdapter.jme.util.JMEGeometryUtils;
import us.ihmc.sensorProcessing.pointClouds.octree.OctreeOccupancyExample;
import bubo.maps.d3.grid.CellProbability_F64;
import bubo.maps.d3.grid.GridMapSpacialInfo3D;
import bubo.maps.d3.grid.impl.OctreeGridMap_F64;

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

public class DisplayOctreeApp extends SimpleApplication
{
   private final String fileName;

   public static void main(String[] args)
   {
      DisplayOctreeApp test1 = new DisplayOctreeApp("../SensorProcessing/data/kinectcloud.txt");
      AppSettings appSettings = new AppSettings(true);
      appSettings.setResolution(1600, 900);
      test1.setSettings(appSettings);
      test1.setShowSettings(false);
      test1.start();
   }

   public DisplayOctreeApp(String fileName)
   {
      this.fileName = fileName;
   }

   @Override
   public void simpleInitApp()
   {
      List<Point3D_F64> cloud = PointCloudTools.readPointCloud(fileName, -1);

      System.out.println("total points: " + cloud.size());
      GridMapSpacialInfo3D spacial = OctreeOccupancyExample.createSpacial();
      OctreeGridMap_F64 map = OctreeOccupancyExample.createMapFromCloud(cloud, spacial, 0);

      Node zUpNode = new Node();
      zUpNode.setLocalRotation(JMEGeometryUtils.getRotationFromJMEToZupCoordinates());

      try
      {
         rootNode.attachChild(zUpNode);
         generateMap(zUpNode, map,spacial);
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

   private void generateMap(Node zUpNode, OctreeGridMap_F64 map, GridMapSpacialInfo3D spacial)
   {
      Iterator<CellProbability_F64> iterator = map.iteratorKnown();
      float r = (float)(spacial.getCellSize()/2.0);

      int i = 0;
      while( iterator.hasNext() ) {
         CellProbability_F64 o = iterator.next();
         Point3D_F64 p = new Point3D_F64();

         spacial.gridToMap(o.x,o.y,o.z,p);
         spacial.mapToCanonical(p,p);
         p.x += r; p.y += r;p.z += r;

         Box box = new Box(new Vector3f((float) p.z, (float) -p.x, (float) -p.y), (float) r, (float) r, (float) r);
         Geometry cube = new Geometry("cell" + i, box);
         Material mat1 = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
         mat1.getAdditionalRenderState().setBlendMode(BlendMode.Alpha);
         cube.setQueueBucket(Bucket.Transparent);
         float prob = (float) o.probability;
         ColorRGBA color = new ColorRGBA(1.0f - prob, prob, 0f, Math.abs(prob-0.5f)/0.5f);
         mat1.setColor("Color", color);
         cube.setMaterial(mat1);
         zUpNode.attachChild(cube);
         i++;
      }
   }

}
