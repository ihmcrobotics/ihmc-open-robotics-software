package us.ihmc.sensorProcessing.pointClouds.shape;

import java.util.Random;

import com.jme3.app.SimpleApplication;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;



public class TestPointCloud extends SimpleApplication
{
   public Vector3f MIN = new Vector3f(-4.675392f, -3.287754f, 0.0f);
   public Vector3f MAX = new Vector3f(5.847956f, 2.126556f, 19.251f);
   public Vector3f CENTER = MAX.add(MIN).mult(0.5f);
   public Vector3f DELTA = MAX.subtract(CENTER);
   public int NUMPOINTS = 2000000;

   public static void main(String[] args)
   {
      TestPointCloud test1 = new TestPointCloud();
      test1.start();
   }

   @Override
   public void simpleInitApp()
   {
      Vector3f[] points = generatePoints(NUMPOINTS);
      ColorRGBA[] colors = generateColors(NUMPOINTS);

      PointCloud generator = new PointCloud(assetManager);


      try
      {
         rootNode.attachChild(generator.generatePointCloudGraph(points, colors,0.75f));
      }
      catch (Exception e)
      {
         this.handleError(e.getMessage(), e);
      }

      cam.setFrustumPerspective(45.0f, ((float) cam.getWidth()) / ((float) cam.getHeight()), 0.05f, 100.0f);
      cam.setLocation(new Vector3f(0, 0, -5));
      cam.lookAtDirection(Vector3f.UNIT_Z, Vector3f.UNIT_Y);

      cam.update();
   }

   public Vector3f[] generatePoints(int numberOfPoints)
   {
      Vector3f[] result = new Vector3f[numberOfPoints];
      Random random = new Random();
      for (int i = 0; i < numberOfPoints; i++)
      {
         float x = CENTER.x + DELTA.x * (random.nextFloat() - random.nextFloat());
         float y = CENTER.y + DELTA.y * (random.nextFloat() - random.nextFloat());
         float z = CENTER.z + DELTA.z * (random.nextFloat() - random.nextFloat());
         result[i] = new Vector3f(x, y, z);

      }

      return result;
   }

   public ColorRGBA[] generateColors(int numberOfPoints)
   {
      ColorRGBA[] result = new ColorRGBA[numberOfPoints];
      Random random = new Random();
      for (int i = 0; i < numberOfPoints; i++)
      {
         result[i] = new ColorRGBA(random.nextFloat(), random.nextFloat(), random.nextFloat(), 1.0f);
      }

      return result;
   }
}
