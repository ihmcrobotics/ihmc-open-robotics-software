package us.ihmc.sensorProcessing.pointClouds.shape;

import georegression.struct.point.Point3D_F64;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Scanner;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;

import us.ihmc.graphics3DAdapter.jme.util.JMEGeometryUtils;
import us.ihmc.utilities.lidar.polarLidar.LidarScan;
import us.ihmc.utilities.lidar.polarLidar.geometry.LidarScanParameters;

import com.jme3.app.SimpleApplication;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;

/**
 * @author Peter Abeles
 */
public class LoadCloudWithPoses extends SimpleApplication
{
   Random rand = new Random();

   public String fileName;

   public static void main(String[] args)
   {
      LoadCloudWithPoses test1 = new LoadCloudWithPoses("D:\\lidarLog_5_1.txt");
      test1.start();
   }

   public LoadCloudWithPoses(String fileName)
   {
      this.fileName = fileName;
   }

   @Override
   public void simpleInitApp()
   {
      List<Point3D_F64>[] clouds = loadPointCloud((int)(40 * 20), 1, true);

      if (true)
      {
         try
         {
            FileWriter fw = new FileWriter("box_10s_h.txt");
            for (Point3D_F64 p : clouds[0])
            {
               fw.write(p.x + " " + p.y + " " + p.z + "\n");
            }
            fw.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

      render(clouds);
   }

   private void render(List<Point3D_F64>[] clouds)
   {
      int total = 0;
      for (List<Point3D_F64> cloud : clouds)
      {
         total += cloud.size();
      }

      Vector3f[] points = new Vector3f[total];
      ColorRGBA[] colors = new ColorRGBA[total];

      int index = 0;
      for (int i = 0; i < clouds.length; i++)
      {
         int c = Color.HSBtoRGB((i / (float) clouds.length), 1.0f, i == 2 ? 1.0f : 1.0f);
         ColorRGBA color = new ColorRGBA(((c >> 16) & 0xFF) / 256.0f, ((c >> 8) & 0xFF) / 256.0f, ((c >> 0) & 0xFF) / 256.0f, 1.0f);

         for (Point3D_F64 p : clouds[i])
         {
            points[index] = new Vector3f((float) p.x, (float) p.y, (float) p.z);
            colors[index] = color;
            index++;
         }
      }

      Node zUpNode = new Node();
      zUpNode.setLocalRotation(JMEGeometryUtils.getRotationFromJMEToZupCoordinates());
      PointCloud generator = new PointCloud(assetManager);

      try
      {
         rootNode.attachChild(zUpNode);
         zUpNode.attachChild(generator.generatePointCloudGraph(points, colors, 1.0f));
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

   private List<Point3D_F64>[] loadPointCloud(int maxScans, int mod, boolean half)
   {
      List<Point3D_F64>[] clouds = new ArrayList[3];
      for (int i = 0; i < clouds.length; i++)
         clouds[i] = new ArrayList<Point3D_F64>();

      try
      {
         int pointsPerSweep = 1081;
         String file = new Scanner(new BufferedReader(new FileReader(fileName))).useDelimiter("\\Z").next();
         LidarScanParameters param = new LidarScanParameters(pointsPerSweep, -2.356194f, 2.356194f, 0, 0, 1, 0, 0, 0, 0, 0, false);

         Transform3D start, end;
         Point3d point;
         float distance;

         List<String> allMatches = new ArrayList<String>();
         Matcher m = Pattern.compile("-?[0-9]+.[0-9]+(E-[0-9])?").matcher(file);
         while (m.find())
         {
            allMatches.add(m.group());
         }
         double[] doubles = new double[allMatches.size()];
         for (int i = 0; i < doubles.length; i++)
            doubles[i] = Float.valueOf(allMatches.get(i));

         int i = 0;
         int scans = 0;
         while (scans < maxScans && i < doubles.length - 1000)
         {
            scans++;

            if (scans % mod != 0)
               continue;

            double[] mx = new double[16];
            for (int j = 0; j < 16; j++)
               mx[j] = doubles[i++];
            start = new Transform3D(mx);

            mx = new double[16];
            for (int j = 0; j < 16; j++)
               mx[j] = doubles[i++];
            end = new Transform3D(mx);

            float[] ranges = new float[pointsPerSweep];
            for (int j = 0; j < pointsPerSweep; j++)
            {
               distance = (float) doubles[i++];
               ranges[j] = distance;
            }

            LidarScan scan = new LidarScan(param, start, end, ranges);

            for (int j = 0; j < scan.size(); j++)
            {
               if (.5 < scan.getRange(j) && scan.getRange(j) < 30.0)
               {
                  Point3d p = scan.getPoint(j);
                  if (!half || j > scan.size() / 2)
                     clouds[0].add(new Point3D_F64(p.x, p.y, p.z));
                  else
                     clouds[1].add(new Point3D_F64(p.x, p.y, p.z));
               }
            }
         }
      }
      catch (FileNotFoundException e)
      {
         throw new RuntimeException(e);
      }
      return clouds;
   }
}
