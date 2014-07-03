package us.ihmc.sensorProcessing.pointClouds.shape;

import georegression.struct.point.Point3D_F64;

import java.awt.Color;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.List;
import java.util.Random;
import java.util.jar.Pack200;

import org.jfree.chart.plot.RainbowPalette;
import org.w3c.dom.css.RGBColor;

import us.ihmc.graphics3DAdapter.jme.util.JMEGeometryUtils;
import us.ihmc.sensorProcessing.pointClouds.octree.OccupancyCell;
import us.ihmc.sensorProcessing.pointClouds.octree.OctreeOccupancyExample;
import us.ihmc.sensorProcessing.pointClouds.octree.OctreeOccupancyMap;
import bubo.io.serialization.DataDefinition;
import bubo.io.serialization.SerializationDefinitionManager;
import bubo.io.text.ReadCsvObjectSmart;
import bubo.ptcloud.Octree;

import com.jme3.app.SimpleApplication;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Box;


public class DisplayPointCloudFileApp extends SimpleApplication
{
   Random rand = new Random(234);

   public String fileName;

   public static void main(String[] args)
   {
      DisplayPointCloudFileApp test1 = new DisplayPointCloudFileApp("data/kinectcloud.txt");
      test1.start();
   }

   public DisplayPointCloudFileApp(String fileName)
   {
      this.fileName = fileName;
   }

   private ColorRGBA generateColors(float value, float colorRange)
   {
      
      Color color = Color.getHSBColor((value%colorRange) / colorRange, 0.85f, 1.0f);
      return new ColorRGBA(color.getRed() / 255.0f, color.getGreen() / 255.0f, color.getBlue() / 255.0f, 1.0f);
   }

   @Override
   public void simpleInitApp()
   {
      List<Point3D_F64> cloud = PointCloudTools.readPointCloud(fileName,-1);

      System.out.println("total points: " + cloud.size());
      OctreeOccupancyMap map = OctreeOccupancyExample.mainMethod(cloud);
      
      Vector3f[] points = new Vector3f[cloud.size()];
      ColorRGBA[] colors = new ColorRGBA[cloud.size()];
      int index = 0;
      for (Point3D_F64 p : cloud)
      {
        // ColorRGBA color = generateColors(new Float(p.z), noColors);
        ColorRGBA color =  map.getLeafColor(p.x, p.y, p.z);
         points[index] = new Vector3f((float) p.z, (float) -p.x, (float) -p.y);
         colors[index] = color;
         index++;
      }


      Node zUpNode = new Node();
      zUpNode.setLocalRotation(JMEGeometryUtils.getRotationFromJMEToZupCoordinates());

      PointCloud generator = new PointCloud(assetManager);

      try
      {
         rootNode.attachChild(zUpNode);
         generateMap(zUpNode, map);
         //zUpNode.attachChild(generator.generatePointCloudGraph(points, colors,0.75f));
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
   }
   private void generateMap(Node zUpNode,OctreeOccupancyMap map){
      int i =0;
      for(Octree o : map.getLeafs()){
         Point3D_F64 p0= o.space.p0;
         Point3D_F64 p1 = o.space.p1;
         p0=p0.plus(p1);
         Box box = new Box(new Vector3f( (float)p0.z/2,(float)-p0.x/2,(float)-p0.y/2),(float)0.025 , (float)0.025,(float) 0.025); 
         Geometry cube = new Geometry("cell"+i, box);
         Material mat1 = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
         OccupancyCell c = o.getUserData();
         mat1.setColor("Color", c.getColor());
         cube.setMaterial(mat1);
         zUpNode.attachChild(cube);
         i++;
      }
   }
   
}
