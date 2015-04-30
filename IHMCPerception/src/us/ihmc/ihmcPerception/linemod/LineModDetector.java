package us.ihmc.ihmcPerception.linemod;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.PriorityQueue;

import javax.vecmath.Vector3d;

import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.UnitConversions;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;

import com.jme3.math.FastMath;

public class LineModDetector
{

   static float epsilon = 1e-5f;
   ArrayList<LineModTemplate> byteFeatures = new ArrayList<>();
   String modelPathInResource;

   public LineModDetector(String modelPathInResource)
   {
      this.modelPathInResource = modelPathInResource;

   }

   @SuppressWarnings("unchecked")
   public void loadFeatures(File file)
   {
      try
      {
         ObjectInputStream output = new ObjectInputStream(new FileInputStream(file));
         byteFeatures = (ArrayList<LineModTemplate>) output.readObject();
         output.close();
      }
      catch (IOException | ClassNotFoundException e)
      {
         e.printStackTrace();
      }
   }

   public void saveFeatures(File file)
   {
      try
      {
         ObjectOutputStream output = new ObjectOutputStream(new FileOutputStream(file));
         output.writeObject(byteFeatures);
         output.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public OrganizedPointCloud renderCloud(double yaw, double pitch, double roll, double distance)
   {
      Render3dObject renderer = new Render3dObject(new File(modelPathInResource));

      renderer.renderImage((float) yaw, (float) pitch, (float) roll, (float) distance);

      return renderer.getPointcloud();
   }

   private int[] makeMaskByZThresholing(OrganizedPointCloud pointCloud, float minDepth)
   {
      int[] mask = new int[pointCloud.width * pointCloud.height];
      final int zFieldOffset = 2;
      final int stepSize = 4;
      for (int i = 0; i < mask.length; i++)
      {
         mask[i] = pointCloud.xyzrgb[i * stepSize + zFieldOffset] < minDepth ? 1 : 0;
      }
      return mask;
   }

   ArrayList<Vector3d> generateTrainingCameraPoses(float maxLevel)
   {
      PriorityQueue<Pair<Integer, int[]>> triangles = new PriorityQueue<Pair<Integer, int[]>>(1,new Comparator<Pair<Integer, int[]>>()
      {
         @Override
         public int compare(Pair<Integer, int[]> o1, Pair<Integer, int[]> o2)
         {
            return o1.first().compareTo(o2.first());
         }
      });

      ArrayList<Vector3d> vertex = new ArrayList<>();

      //first triangle
      vertex.add(new Vector3d(1, 0, 0));
      vertex.add(new Vector3d(0, 1, 0));
      vertex.add(new Vector3d(0, 0, 1));
      vertex.add(new Vector3d(-1, 0, 0));
      vertex.add(new Vector3d(0, -1, 0));
      //vertex.add(new Vector3d(0, 0, -1));
      triangles.add(new Pair<Integer, int[]>(0, new int[] { 0, 1, 2 }));
      triangles.add(new Pair<Integer, int[]>(0, new int[] { 1, 3, 2 }));
      triangles.add(new Pair<Integer, int[]>(0, new int[] { 3, 4, 2 }));
      triangles.add(new Pair<Integer, int[]>(0, new int[] { 0, 4, 2 }));

      //start fragmentation
      while (triangles.peek().first() < maxLevel)
      {
         Pair<Integer, int[]> largestTriangle = triangles.remove();
         int currentLevel = largestTriangle.first();
         int[] oldVertex = largestTriangle.second();
         Vector3d p0 = new Vector3d();
         Vector3d p1 = new Vector3d();
         Vector3d p2 = new Vector3d();
         p0.add(vertex.get(oldVertex[0]), vertex.get(oldVertex[1]));
         p0.normalize();
         p1.add(vertex.get(oldVertex[1]), vertex.get(oldVertex[2]));
         p1.normalize();
         p2.add(vertex.get(oldVertex[0]), vertex.get(oldVertex[2]));
         p2.normalize();

         int[] newVertex = new int[] { vertex.size(), vertex.size() + 1, vertex.size() + 2 };
         vertex.add(p0);
         vertex.add(p1);
         vertex.add(p2);

         triangles.add(new Pair<Integer, int[]>(currentLevel + 1, new int[] { newVertex[0], newVertex[2], oldVertex[0] }));
         triangles.add(new Pair<Integer, int[]>(currentLevel + 1, new int[] { newVertex[0], newVertex[1], oldVertex[1] }));
         triangles.add(new Pair<Integer, int[]>(currentLevel + 1, new int[] { newVertex[1], newVertex[2], oldVertex[2] }));
         triangles.add(new Pair<Integer, int[]>(currentLevel + 1, new int[] { newVertex[0], newVertex[1], newVertex[2] }));

      }

      Collections.sort(vertex, new Comparator<Vector3d>()
      {

         @Override
         public int compare(Vector3d o1, Vector3d o2)
         {
            if (Double.compare(o1.x, o2.x) == 0)
            {
               if (Double.compare(o1.y, o2.y) == 0)
               {
                  return Double.compare(o1.z, o2.z);
               }
               else
               {
                  return Double.compare(o1.y, o2.y);
               }
            }
            else
            {
               return Double.compare(o1.x, o2.x);
            }

         };
      });

      return vertex;
   }

   public void trainModelFromRenderedImagesSphere(int coarseLevel)
   {
      long millitimeStartTraining = System.currentTimeMillis();
      byteFeatures.clear();

      ArrayList<Vector3d> viewPoints = generateTrainingCameraPoses(coarseLevel);

      for (Vector3d viewpoint : viewPoints)
      {
         float yaw = (float) Math.atan2(viewpoint.y, viewpoint.x);
         float pitch = (float) Math.asin(viewpoint.z);
         if (pitch > FastMath.PI / 3)
            continue;
         for (float roll = -FastMath.PI / 12; roll <= FastMath.PI / 12; roll += FastMath.PI / 12)
         {
            System.out.println("ypr " + yaw + " " + pitch + " " + roll);
            trainSingleDetector(yaw, pitch, roll, 0.0);
         }

      }

      System.out.println("\ntraining time :" + (System.currentTimeMillis() - millitimeStartTraining) + " millisecond for " + byteFeatures.size() + " images");

   }

   void trainSingleDetector(double yaw, double pitch, double roll, double distance)
   {
      OrganizedPointCloud cloud = renderCloud(yaw, pitch, roll, distance);
      int[] mask = makeMaskByZThresholing(cloud, 1.5f);
      LineModTemplate template = LineModInterface.trainTemplateBytes(cloud, mask);
      template.mask = mask;
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setEuler(roll, pitch, yaw);
      transform.setTranslation(0, 0, distance);
      template.transform = transform;
      byteFeatures.add(template);
   }

   public LineModDetection detectObjectAndEstimatePose(OrganizedPointCloud testCloud, ArrayList<LineModDetection> detectionsToPack)
   {
      return detectObjectAndEstimatePose(testCloud, detectionsToPack, false, false);
   }

   public LineModDetection detectObjectAndEstimatePose(OrganizedPointCloud testCloud, ArrayList<LineModDetection> detectionsToPack, boolean averaging,
         boolean nonMaximalSupression)
   {

      long millitimeStartDetection = System.currentTimeMillis();
      ArrayList<LineModDetection> detections = LineModInterface.matchTemplatesBytes(testCloud, byteFeatures, averaging, nonMaximalSupression);
      if (detectionsToPack != null)
         detectionsToPack.addAll(detections);
      System.out.println("detection time :" + (System.currentTimeMillis() - millitimeStartDetection) + " millisecond");

      int maxi = -1;
      for (int i = 0; i < detections.size(); i++)
      {
         LineModDetection currentDetection = detections.get(i);
         currentDetection.template = byteFeatures.get(currentDetection.template_id);
         if (maxi < 0 || detections.get(maxi).score < currentDetection.score)
            maxi = i;
      }
      if (maxi < 0)
         return null;
      else
         return detections.get(maxi);
   }

   public static void main(String[] arg)
   {

      LineModDetector detector = new LineModDetector("/examples/drill/drill.obj");
      File featureFile = new File("FeatureSphare3x4.dat");
      if (featureFile.exists())
      {
         System.out.println("loading feature from disk");
         detector.loadFeatures(featureFile);
      }
      else
      {
         System.out.println("trainign new feature");
         detector.trainModelFromRenderedImagesSphere(3);
         detector.saveFeatures(featureFile);
      }

      for (int i = 0; i < 10; i++)
      {
         float groundTruthAngle = 0.52f; 
         float distancePerturbation = (float) Math.random() * 0.2f;
         OrganizedPointCloud testCloud = detector.renderCloud(groundTruthAngle, 0.0f, 0.0f, distancePerturbation);
         ArrayList<LineModDetection> detections = new ArrayList<>();
         LineModDetection bestDetection = detector.detectObjectAndEstimatePose(testCloud, detections);
         Vector3d rollPitchYaw = new Vector3d();
         bestDetection.template.transform.getEulerXYZ(rollPitchYaw);
         System.out.println("estimated " + rollPitchYaw.getZ()/UnitConversions.DEG_TO_RAD + " groundtruth " + groundTruthAngle/UnitConversions.DEG_TO_RAD);
      }
   }
}
