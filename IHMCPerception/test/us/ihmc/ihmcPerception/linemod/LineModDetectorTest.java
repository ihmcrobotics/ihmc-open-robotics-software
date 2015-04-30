package us.ihmc.ihmcPerception.linemod;

import static org.junit.Assert.*;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;

import javax.imageio.ImageIO;
import javax.vecmath.Vector3d;

import org.junit.Test;

import com.jme3.math.FastMath;

import us.ihmc.utilities.math.UnitConversions;
import us.ihmc.utilities.math.geometry.RotationFunctions;

public class LineModDetectorTest
{

   @Test
   public void testGenerateVertexes()
   {
      LineModDetector detector = new LineModDetector(null);
      ArrayList<Vector3d> vertexes = detector.generateTrainingCameraPoses(3);
//      for (Vector3d vector3d : vertexes)
//         System.out.println(vector3d.x + " " + vector3d.y + " " + vector3d.z);
      org.junit.Assert.assertEquals(257,vertexes.size());
   }

   @Test
   public void trainOneTestOne() throws IOException
   {
      LineModDetector detector = new LineModDetector("/examples/drill/drill.obj");
      detector.trainSingleDetector(0.0, 0.0, 0.0, 0.0);

      OrganizedPointCloud cloud = detector.renderCloud(0.0, 0.0, 0.0, 0.1);
      ImageIO.write(cloud.getRGBImage(), "png", new File("testRGB.png"));
      LineModDetection bestDetection = detector.detectObjectAndEstimatePose(cloud, null);
      assertEquals(cloud.getRGBImage().getWidth() / 2, bestDetection.x + bestDetection.template.region.width/2, 20);
      assertEquals(cloud.getRGBImage().getHeight() / 2, bestDetection.y + bestDetection.template.region.height, 20);
//      System.out.println("score:"+bestDetection.score);
      assertTrue(bestDetection.score> 0.98);
   }
   
   @Test
   public void testFeatureSaveLoad()
   {
      LineModDetector detector = new LineModDetector("/examples/drill/drill.obj");
      detector.trainSingleDetector(0.0, 0.0, 0.0, 0.0);
      try
      {
         File tempFile = File.createTempFile("LineModFeature", "data");
         detector.saveFeatures(tempFile);
         detector.byteFeatures.clear();
         detector.loadFeatures(tempFile);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      OrganizedPointCloud cloud = detector.renderCloud(0.0, 0.0, 0.0, 0.0);
      LineModDetection bestDetection = detector.detectObjectAndEstimatePose(cloud, null);
      assertEquals(cloud.getRGBImage().getWidth() / 2, bestDetection.x + bestDetection.template.region.width/2, 20);
      assertEquals(cloud.getRGBImage().getHeight() / 2, bestDetection.y + bestDetection.template.region.height, 20);;
      assertTrue(bestDetection.score> 0.98);
      System.out.println("score:"+bestDetection.score);
   }

   @Test
   public void testYawAngles() 
   {
      LineModDetector detector = new LineModDetector("/examples/drill/drill.obj");
      int nYaws = 24;
      for(int i=0;i<nYaws;i++)
      {
         detector.trainSingleDetector(2.0*Math.PI*i/nYaws, 0.0, 0.0, 0.0);
         System.out.print(".");
      }
      
      for(double groundTruthYaw=0;groundTruthYaw<FastMath.TWO_PI;groundTruthYaw+=Math.PI/60.0)
      {
              OrganizedPointCloud cloud = detector.renderCloud(groundTruthYaw, 0.0, 0.0, 0.2);
              LineModDetection bestDetection = detector.detectObjectAndEstimatePose(cloud, null,false,true);
              if(bestDetection!=null)
              {
                      Vector3d yawPitchRoll = new Vector3d();
                      bestDetection.template.transform.getEulerXYZ(yawPitchRoll);
                      double estimatedYaw= yawPitchRoll.getZ();
                      double yawError = Math.min(Math.abs(estimatedYaw-groundTruthYaw),Math.PI*2-Math.abs(estimatedYaw-groundTruthYaw));
//                      System.out.println(yawError/UnitConversions.DEG_TO_RAD);
                      assertTrue(yawError < 20.0*UnitConversions.DEG_TO_RAD);
//                      System.out.println("groundTruth = " + groundTruthYaw/UnitConversions.DEG_TO_RAD + " estimated = "+estimatedYaw/UnitConversions.DEG_TO_RAD + " " + "confidence " + bestDetection.score);
              }
              else
              {
                 fail("no detection");
              }
      }
   }
}
