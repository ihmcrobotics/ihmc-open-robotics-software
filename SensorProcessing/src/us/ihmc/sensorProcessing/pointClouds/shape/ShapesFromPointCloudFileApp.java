package us.ihmc.sensorProcessing.pointClouds.shape;

import bubo.io.serialization.DataDefinition;
import bubo.io.serialization.SerializationDefinitionManager;
import bubo.io.text.ReadCsvObjectSmart;
import bubo.ptcloud.FactoryPointCloudShape;
import bubo.ptcloud.PointCloudShapeFinder;
import bubo.ptcloud.alg.ConfigSchnabel2007;
import bubo.ptcloud.wrapper.ConfigMergeShapes;
import bubo.ptcloud.wrapper.ConfigSurfaceNormals;
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
public class ShapesFromPointCloudFileApp extends SimpleApplication
{
   Random rand = new Random(234);

   public String fileName;

   public static void main(String[] args)
   {
      ShapesFromPointCloudFileApp test1 = new ShapesFromPointCloudFileApp("/home/pja/Downloads/output.txt");
      test1.start();
   }

   public ShapesFromPointCloudFileApp(String fileName)
   {
      this.fileName = fileName;
   }

   @Override
   public void simpleInitApp()
   {
      List<Point3D_F64> cloud = readPointCloud(1000000);

      ConfigSchnabel2007 configRansac = ConfigSchnabel2007.createDefault(100, 0.3, 0.1, 0.1);
      configRansac.minModelAccept = 200;
      configRansac.octreeSplit = 300;

      ConfigSurfaceNormals configSurface = new ConfigSurfaceNormals(10, 30, Double.MAX_VALUE);
      ConfigMergeShapes configMerge = new ConfigMergeShapes(0.6, 0.9);

      PointCloudShapeFinder shapeFinder = FactoryPointCloudShape.ransacOctree(configSurface, configRansac, configMerge);

      shapeFinder.process(cloud, null);

      List<PointCloudShapeFinder.Shape> found = shapeFinder.getFound();

      List<Point3D_F64> unmatched = new ArrayList<Point3D_F64>();
      shapeFinder.getUnmatched(unmatched);

      System.out.println("Unmatched points " + unmatched.size());
      System.out.println("total shapes found: " + found.size());
      int total = 0;
      for (PointCloudShapeFinder.Shape s : found)
      {
         System.out.println("  " + s.type + "  points = " + s.points.size());
         System.out.println("           " + s.parameters);
         total += s.points.size();
      }

      Vector3f[] points = new Vector3f[total];
      ColorRGBA[] colors = new ColorRGBA[total];

      int index = 0;
      for (PointCloudShapeFinder.Shape s : found)
      {
         float r = rand.nextFloat();
         float g = rand.nextFloat();
         float b = rand.nextFloat();

         // make sure it is bright enough to see
         float n = (r + g + b) / 3.0f;

         if (n < 0.5f)
         {
            r *= 0.5f / n;
            g *= 0.5f / n;
            b *= 0.5f / n;
         }


         ColorRGBA color = new ColorRGBA(r, g, b, 1.0f);
         System.out.println(" color " + r + " " + g + " " + b);

         for (Point3D_F64 p : s.points)
         {
            points[index] = new Vector3f((float) p.x, (float) p.y, (float) p.z);
            colors[index] = color;
            index++;
         }
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
