package us.ihmc.robotics.hyperCubeTree;

import us.ihmc.tools.random.RandomTools;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.Random;

public class HyperCubeTreeStaticTestingUtilities
{

   public static void shootTreeIntoSphere(Octree tree, double radius, double shootingDistance, double xCenter, double yCenter, double zCenter, int numberOfLidarBullets)
   {
      Random rand = new Random(-1985L);
      double[] centerAsArray = new double[] { xCenter, yCenter, zCenter };
      double[] startPointAsArray = new double[3];
      double[] endPointAsArray = new double[3];
      double[] vectorAsArray = new double[3];
      double radiusToShootFrom = shootingDistance;
      for (int k = 0; k < numberOfLidarBullets; k++)
      {
         Vector3d vector3d = RandomTools.generateRandomVector(rand, 1.0);
         vector3d.get(vectorAsArray);
         for (int j = 0; j < 3; j++)
         {
            startPointAsArray[j] = vectorAsArray[j] * radiusToShootFrom + centerAsArray[j];
            endPointAsArray[j] = vectorAsArray[j] * radius + centerAsArray[j];
         }
         Point3d start = new Point3d(startPointAsArray);
         Point3d end = new Point3d(endPointAsArray);
         tree.putLidarAtGraduallyMoreAccurateResolution(start, end);
      }
//      tree.mergeIfPossible();
   }

   public static void fillOctreeDefaultTest(HyperCubeTree<Boolean, Void> tree, double d1, double d2, double d3, double d4)
   {
      tree.put(new double[] { d1, d3, d1 }, true);
      tree.put(new double[] { d1, d3, d1 }, true);
      tree.put(new double[] { d2, d3, d1 }, false);
      tree.put(new double[] { d1, d4, d1 }, true);
      tree.put(new double[] { d2, d4, d1 }, true);
      tree.put(new double[] { d1, d3, d2 }, true);
      tree.put(new double[] { d2, d3, d2 }, false);
      tree.put(new double[] { d1, d4, d2 }, true);
      tree.put(new double[] { d2, d4, d2 }, true);
   
      //LLL Oct
      //LLL Oct left intentionally undefined.
   
      //HLL Oct 
      tree.put(new double[] { d3, d1, d1 }, true);
   
      //HHL Oct         
      tree.put(new double[] { d3, d3, d1 }, true);
   
      //LLH Oct   
      tree.put(new double[] { d1, d1, d3 }, true);
      tree.put(new double[] { d2, d1, d3 }, false);
      tree.put(new double[] { d1, d2, d3 }, true);
      tree.put(new double[] { d2, d2, d3 }, false);
      tree.put(new double[] { d1, d1, d4 }, true);
      tree.put(new double[] { d2, d1, d4 }, false);
      tree.put(new double[] { d1, d2, d4 }, true);
      tree.put(new double[] { d2, d2, d4 }, false);
   
      //HLH Oct
      tree.put(new double[] { d3, d1, d3 }, false);
      tree.put(new double[] { d4, d1, d3 }, true);
      tree.put(new double[] { d3, d2, d3 }, false);
      tree.put(new double[] { d4, d2, d3 }, true);
      tree.put(new double[] { d3, d1, d4 }, false);
      tree.put(new double[] { d4, d1, d4 }, false);
      tree.put(new double[] { d3, d2, d4 }, false);
      tree.put(new double[] { d4, d2, d4 }, false);
   
      //LHH Oct
      tree.put(new double[] { d1, d3, d3 }, false);
   
      //HHH Oct
      tree.put(new double[] { d3, d3, d3 }, false);
      tree.put(new double[] { d4, d3, d3 }, true);
      tree.put(new double[] { d3, d4, d3 }, false);
      tree.put(new double[] { d4, d4, d3 }, false);
      tree.put(new double[] { d3, d3, d4 }, false);
      tree.put(new double[] { d4, d3, d4 }, false);
      tree.put(new double[] { d3, d4, d4 }, false);
      tree.put(new double[] { d4, d4, d4 }, false);
   }



}
