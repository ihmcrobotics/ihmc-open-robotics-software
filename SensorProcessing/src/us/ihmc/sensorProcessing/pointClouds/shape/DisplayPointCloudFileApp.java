package us.ihmc.sensorProcessing.pointClouds.shape;

import bubo.io.serialization.DataDefinition;
import bubo.io.serialization.SerializationDefinitionManager;
import bubo.io.text.ReadCsvObjectSmart;
import com.jme3.app.SimpleApplication;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import georegression.struct.point.Point3D_F64;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * @author Peter Abeles
 */
public class DisplayPointCloudFileApp extends SimpleApplication
{
   Random rand = new Random(234);

   public String fileName;

   public static void main(String[] args)
   {
      DisplayPointCloudFileApp test1 = new DisplayPointCloudFileApp("/home/pja/Downloads/output.txt");
      test1.start();
   }

   public DisplayPointCloudFileApp(String fileName)
   {
      this.fileName = fileName;
   }

   @Override
   public void simpleInitApp()
   {
      List<Point3D_F64> cloud = readPointCloud(1000000);

      System.out.println("total points: " + cloud.size());

      Vector3f[] points = new Vector3f[cloud.size()];
      ColorRGBA[] colors = new ColorRGBA[cloud.size()];

      ColorRGBA color = new ColorRGBA(1, 0, 0, 1.0f);
      int index = 0;
      for (Point3D_F64 p : cloud )
      {
         points[index] = new Vector3f((float) p.x, (float) p.y, (float) p.z);
         colors[index] = color;
         index++;
      }


      PointCloud generator = new PointCloud(assetManager);

      try
      {
         rootNode.attachChild(generator.generatePointCloudGraph(points, colors));
      }
      catch (Exception e)
      {
         this.handleError(e.getMessage(), e);
      }

      cam.setFrustumPerspective(45.0f, ((float) cam.getWidth()) / ((float) cam.getHeight()), 0.05f, 100.0f);
      cam.setLocation(new Vector3f(0, 0, -5));
      cam.lookAtDirection(Vector3f.UNIT_Z, Vector3f.UNIT_Y);
      cam.update();
      flyCam.setMoveSpeed(25);
   }

   private List<Point3D_F64> readPointCloud( int maxLines )
   {
      List<Point3D_F64> cloud = new ArrayList<Point3D_F64>();
      SerializationDefinitionManager manager = new SerializationDefinitionManager();
      manager.addDefinition(new DataDefinition("point3d",Point3D_F64.class,"x","y","z"));

      try
      {
         FileInputStream in = new FileInputStream(fileName);
         ReadCsvObjectSmart<Point3D_F64> reader = new ReadCsvObjectSmart<Point3D_F64>(in,manager,"point3d");


         Point3D_F64 pt = new Point3D_F64();
         int count = 0;
         while(  reader.nextObject(pt) != null && count++ < maxLines) {
            cloud.add(pt.copy());
         }

      } catch (FileNotFoundException e)
      {
         throw new RuntimeException(e);
      } catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      return cloud;
   }
}
