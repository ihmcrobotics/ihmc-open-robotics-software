package us.ihmc.sensorProcessing.concaveHull;

import georegression.struct.point.Point3D_F64;

import java.awt.Color;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import us.ihmc.graphics3DAdapter.jme.util.JMEGeometryUtils;
import us.ihmc.sensorProcessing.pointClouds.shape.PointCloud;
import us.ihmc.sensorProcessing.pointClouds.shape.ShapeTranslator;
import bubo.io.serialization.DataDefinition;
import bubo.io.serialization.SerializationDefinitionManager;
import bubo.io.text.ReadCsvObjectSmart;

import com.jme3.app.SimpleApplication;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;

/**
 * @author Peter Abeles
 */
public class DisplayPointCloudFileApp extends SimpleApplication
{
   Random rand = new Random(234);

   public String fileName;
   public String fileName2;

   public static void main(String[] args)
   {
      ///home/unknownid/workspace/SensorProcessing/output.txt
	   DisplayPointCloudFileApp test1 = new DisplayPointCloudFileApp("../SensorProcessing/data/kinectcloud.txt","../SensorProcessing/data/kinectcloud.txt");
	  
      test1.start();
   }

   public DisplayPointCloudFileApp(String fileName1,String fileName2)
   {
      this.fileName = fileName1;
      this.fileName2 = fileName2;
   }

   private ColorRGBA generateColors(float value, float colorRange)
   {
      Color color = Color.getHSBColor((value%colorRange) / colorRange, 0.85f, 1.0f);
      return new ColorRGBA(color.getRed() / 255.0f, color.getGreen() / 255.0f, color.getBlue() / 255.0f, 1.0f);
   }

   @Override
   public void simpleInitApp()
   {
	   
	   Node zUpNode = new Node();
	      zUpNode.setLocalRotation(JMEGeometryUtils.getRotationFromJMEToZupCoordinates());
	   
      List<Point3D_F64> cloud = readPointCloud(1000000,fileName);
      
      
     ShapeTranslator shapeTranslator = new ShapeTranslator(this);
//      PlaneGeneral3D_F64 plane = new PlaneGeneral3D_F64(0,1,0,2);
//     
//      shapeTranslator.createPolygon(plane,cloud, ColorRGBA.Cyan, zUpNode);
//      System.out.println("total points: " + cloud.size());

      Vector3f[] points = new Vector3f[cloud.size()*2];
      ColorRGBA[] colors = new ColorRGBA[cloud.size()*2];
      int index = 0;
      for (Point3D_F64 p : cloud)
      {
         ColorRGBA color = generateColors(new Float(2.3), 0.25f);
         points[index] = new Vector3f((float) p.x, (float) p.y, (float) p.z);
         colors[index] = color;
         index++;
      }
      cloud = readPointCloud(1000000, fileName2);
      for (Point3D_F64 p : cloud)
      {
         ColorRGBA color = generateColors(new Float(7.9), 0.25f);
         points[index] = new Vector3f((float) p.x, (float) p.y, (float) p.z);
         colors[index] = color;
         index++;
      }
//      List<Point3D_F64> eigen = readPointCloud(3,fileName2);
//      System.out.println(eigen.size());
//      for (int i = 0; i < eigen.size(); i++)
//      {
//         Vector3f start = new Vector3f(0.0f,0.0f,0.0f);
//         Vector3f end = new Vector3f((float) eigen.get(i).x, (float) eigen.get(i).y, (float) eigen.get(i).z);
//         System.out.println(end.x+" "+end.y+" "+end.z);
//         zUpNode.attachChild(shapeTranslator.drawLine(start, end));
//     
//      }
//      Vector3f start = new Vector3f(0.0f,0.0f,0.0f);
//      Vector3f end = new Vector3f(8.0f, 0.0f, 0.0f);
//      zUpNode.attachChild(shapeTranslator.drawLine(start, end));
//      end = new Vector3f(0.0f,8.0f,0.0f);
//      zUpNode.attachChild(shapeTranslator.drawLine(start, end));
//      end = new Vector3f(0.0f,0.0f,8.0f);
//      zUpNode.attachChild(shapeTranslator.drawLine(start, end));

      PointCloud generator = new PointCloud(assetManager);

      try
      {
         rootNode.attachChild(zUpNode);
         zUpNode.attachChild(generator.generatePointCloudGraph(points, colors,0.75f));
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

   private List<Point3D_F64> readPointCloud(int maxLines, String filename)
   {
      List<Point3D_F64> cloud = new ArrayList<Point3D_F64>();
      SerializationDefinitionManager manager = new SerializationDefinitionManager();
      manager.addDefinition(new DataDefinition("point3d", Point3D_F64.class, "x", "y", "z"));

      try
      {
         FileInputStream in = new FileInputStream(filename);
         ReadCsvObjectSmart<Point3D_F64> reader = new ReadCsvObjectSmart<Point3D_F64>(in, manager, "point3d");


         Point3D_F64 pt = new Point3D_F64();
         int count = 0;
         while ((reader.nextObject(pt) != null) && (count++ < maxLines))
         {
        	 //if(pt.norm()<2.25 && pt.z<1 && pt.x>0.4)   
        	 cloud.add(pt.copy());
         }

      }
      catch (FileNotFoundException e)
      {
         throw new RuntimeException(e);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      return cloud;
   }
}
