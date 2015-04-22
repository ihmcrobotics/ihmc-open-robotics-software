package us.ihmc.ihmcPerception.linemod;

import java.io.File;
import java.util.ArrayList;

import org.jfree.util.UnitType;

import us.ihmc.utilities.math.UnitConversions;

import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;

public class LineModDetector
{
   
   ArrayList<byte[]>  byteFeatures = new ArrayList<>();
   ArrayList<Float> poses = new ArrayList<>();
   String modelPathInResource;
   
   
   public float getPose(int i)
   {
      return poses.get(i);
   }

   public LineModDetector(String modelPathInResource)
   {
      this.modelPathInResource = modelPathInResource;

   }
   
   private OrganizedPointCloud renderCloud(float yawAngle, Vector3f translation)
   {
      Render3dObject renderer = new Render3dObject(new File(modelPathInResource));

      final Quaternion qX = new Quaternion();
      qX.fromAngles(FastMath.PI / 2, 0.0f, 0.0f);
      final Quaternion qYaw = new Quaternion();
      qYaw.fromAngles(0, 0,yawAngle);
      final Quaternion qTilt = new Quaternion();
      qTilt.fromAngles(FastMath.PI / 4, 0.0f, 0.0f);
      
      Transform transform = new Transform(translation,qTilt.mult(qYaw.mult(qX)));
      renderer.renderImage(transform);

      return renderer.getPointcloud();
   }
   
   private int[] makeMaskByZThresholing(OrganizedPointCloud pointCloud, float minDepth)
   {
      int[] mask = new int[pointCloud.width*pointCloud.height];
      final int zFieldOffset=2;
      final int stepSize=4;
      for(int i=0;i<mask.length;i++)
      {
         mask[i] = pointCloud.xyzrgb[i*stepSize+zFieldOffset]<minDepth?1:0;
      }
      return mask;
   }
   
   public void trainModelFromRenderedImages(int numberTemplates)
   {
      System.out.println("Initializing " + numberTemplates + " templates");
      byteFeatures.clear();
      long millitimeStartTraining=System.currentTimeMillis();
      for(int i=0;i<numberTemplates;i++)
      {
         float angle = (float)Math.PI*2*i/numberTemplates;
         OrganizedPointCloud pointCloud = renderCloud(angle,new Vector3f());
         int[] mask = makeMaskByZThresholing(pointCloud, 1.5f);
         byte[] data=LineModInterface.trainTemplateBytes(pointCloud, mask);
         poses.add(angle);
         byteFeatures.add(data);
         System.out.print(i+"..");
      }
      System.out.println("\ntraining time :" + (System.currentTimeMillis()-millitimeStartTraining)+ " millisecond for " + numberTemplates + " images");
   }
   
   public int detectObjectAndEstimatePose(OrganizedPointCloud testCloud, ArrayList<LineModDetection> detectionsToPack) 
   {

      long millitimeStartDetection=System.currentTimeMillis();
      detectionsToPack.addAll(LineModInterface.matchTemplatesBytes(testCloud, byteFeatures));
      System.out.println("detection time :" + (System.currentTimeMillis()-millitimeStartDetection) + " millisecond");
      
      int maxi = 0;
      for(int i=0;i<detectionsToPack.size();i++)
      {
         LineModDetection currentDetection = detectionsToPack.get(i);
         currentDetection.yaw = poses.get(i);
         if(detectionsToPack.get(maxi).score < currentDetection.score)
            maxi=i;
      }
      
      //debug msg
      for(int i=0;i<detectionsToPack.size();i++)
      {
         if(i==maxi)
                 System.out.print("*");
         System.out.print(String.format("%d(%2.0f) ", (int)(poses.get(i)/UnitConversions.DEG_TO_RAD),detectionsToPack.get(i).score*100));
      }
      System.out.println("");

      return maxi;
   }

   
   public static void main(String[] arg)
   {

      LineModDetector detector = new LineModDetector("/examples/drill/drill.obj");
      detector.trainModelFromRenderedImages(24);

      
      for(int i=0;i<10;i++)
      {
         float groundTruthAngle = (float)(Math.PI*2*Math.random());
         Vector3f perturbation = new Vector3f((float)Math.random()*0.2f, (float)Math.random()*0.2f, (float)Math.random()*0.2f);
         OrganizedPointCloud testCloud = detector.renderCloud(groundTruthAngle,perturbation);
         ArrayList<LineModDetection> detections=new ArrayList<>();
         int bestDetectionIndex= detector.detectObjectAndEstimatePose(testCloud, detections);
         float estimatedAngle = detections.get(bestDetectionIndex).yaw;
         System.out.println(
                  " groundTruth " + groundTruthAngle/UnitConversions.DEG_TO_RAD + 
                  " estimated " + estimatedAngle/UnitConversions.DEG_TO_RAD + 
                  " error " +  (estimatedAngle-groundTruthAngle)/UnitConversions.DEG_TO_RAD);
      }
   }
}
