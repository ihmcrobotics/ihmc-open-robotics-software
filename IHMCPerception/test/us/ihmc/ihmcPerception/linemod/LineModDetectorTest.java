package us.ihmc.ihmcPerception.linemod;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import javax.imageio.ImageIO;

import org.junit.Test;

import com.jme3.math.FastMath;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.tools.UnitConversions;

@ContinuousIntegrationPlan(categories = IntegrationCategory.EXCLUDE) // Doesn't work right on all machines
public class LineModDetectorTest
{

   static final String modelFile="drill_DCS551/drillUI.obj";
//   static final String modelFile=""/examples/drill/drill.obj";
         
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout=3000)
   public void testGenerateVertexes()
   {
      LineModDetector detector = new LineModDetector(null);
      ArrayList<Vector3D> vertexes = detector.generateTrainingCameraPoses(3);
//      for (Vector3d vector3d : vertexes)
//         System.out.println(vector3d.x + " " + vector3d.y + " " + vector3d.z);
      org.junit.Assert.assertEquals(257,vertexes.size());
   }

   @ContinuousIntegrationTest(estimatedDuration = 3.0)
   @Test(timeout=3000)
   public void trainOneTestOne() throws IOException
   {
      LineModDetector detector = new LineModDetector(modelFile);
      double angle = Math.PI/6;
      detector.trainSingleDetector(angle, 0.0, 0.0, 1.0);

      OrganizedPointCloud cloud = detector.renderCloud(angle, 0.0, 0.0, 1.0);
      LineModDetection bestDetection = detector.detectObjectAndEstimatePose(cloud, null);
      assertTrue("fail to detect", bestDetection!=null);
      
      BufferedImage image = cloud.getRGBImage();
      detector.drawDetectionOnImage(bestDetection, image);
      ImageIO.write(image, "png", new File("testRGB.png"));
      
      //ensure location is correct
      assertEquals(cloud.getRGBImage().getWidth() / 2, bestDetection.x + bestDetection.template.region.width/2, 20);
      assertEquals(cloud.getRGBImage().getHeight() / 2, bestDetection.y , 20);

      //score is good
      System.out.println("score:"+bestDetection.score);
      assertTrue(bestDetection.score> 0.97);

      //adverserial cloud should receive low score
      OrganizedPointCloud cloudAdverse = detector.renderCloud(angle+Math.PI/2.0, 0.0, 0.0, 1.0);
      ImageIO.write(cloudAdverse.getRGBImage(), "png", new File("testRGBadv.png"));
      LineModDetection bestDetectionAdverse = detector.detectObjectAndEstimatePose(cloudAdverse, null);
      if(bestDetectionAdverse!=null)
      {
         System.out.println("adverse score:"+bestDetectionAdverse.score);
         assertTrue(bestDetectionAdverse.score< 0.95);
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 3000)
   public void trainOneTestOneScaled() throws IOException
   {
      LineModDetector detector = new LineModDetector(modelFile);
      double angle = Math.PI/6;
      detector.trainSingleDetector(angle, 0.0, 0.0, 1.0);

      OrganizedPointCloud cloud = detector.renderCloud(angle, 0.0, 0.0, 1.5);
      LineModDetection bestDetection = detector.detectObjectAndEstimatePose(cloud, null);
      
      BufferedImage image = cloud.getRGBImage();
      if(bestDetection!=null)
      {
         detector.drawDetectionOnImage(bestDetection, image);
      }
      ImageIO.write(image, "png", new File("detect-scaled.png"));
      
      
      //ensure location is correct
      assertEquals(cloud.getRGBImage().getWidth() / 2, bestDetection.x + bestDetection.getScaledWidth()/2, 20);
      assertEquals(cloud.getRGBImage().getHeight() / 2, bestDetection.y, 20);

      //score is good
      System.out.println("score:"+bestDetection.score);
      assertTrue(bestDetection.score> 0.94);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 5000)
   public void testFeatureSaveLoad()
   {
      LineModDetector detector = new LineModDetector(modelFile);
      detector.trainSingleDetector(0.0, 0.0, 0.0, 1.0);
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

      OrganizedPointCloud cloud = detector.renderCloud(0.0, 0.0, 0.0, 1.0);
      LineModDetection bestDetection = detector.detectObjectAndEstimatePose(cloud, null);
      assertEquals(cloud.getRGBImage().getWidth() / 2, bestDetection.x + bestDetection.template.region.width/2, 20);
      assertEquals(cloud.getRGBImage().getHeight() / 2, bestDetection.y, 20);;
      assertTrue(bestDetection.score> 0.98);
      System.out.println("score:"+bestDetection.score);
   }

   @ContinuousIntegrationTest(estimatedDuration = 3.0)
   @Test(timeout=120000)
   public void testYawAngles() 
   {
      LineModDetector detector = new LineModDetector(modelFile);
      int nYaws = 36;
      for(int i=0;i<nYaws;i++)
      {
         detector.trainSingleDetector(2.0*Math.PI*i/nYaws, 0.0, 0.0, 1.0);
         System.out.print(".");
      }

      
      for(double groundTruthYaw=0;groundTruthYaw<FastMath.TWO_PI;groundTruthYaw+=Math.PI/60.0)
      {
              OrganizedPointCloud cloud = detector.renderCloud(groundTruthYaw, 0.0, 0.0, 1.2);
              LineModDetection bestDetection = detector.detectObjectAndEstimatePose(cloud, null,true,false);
              if(bestDetection!=null)
              {
                      Vector3D rollPitchYaw = new Vector3D();
                      bestDetection.template.transform.getRotationEuler(rollPitchYaw);
                      double estimatedYaw= rollPitchYaw.getZ();
                      double yawError = Math.min(Math.abs(estimatedYaw-groundTruthYaw),Math.PI*2-Math.abs(estimatedYaw-groundTruthYaw));
                      System.out.println(yawError/UnitConversions.DEG_TO_RAD);
                      System.out.println("groundTruth = " + groundTruthYaw/UnitConversions.DEG_TO_RAD + " estimated = "+estimatedYaw/UnitConversions.DEG_TO_RAD + " " + "confidence " + bestDetection.score);
                      assertTrue(yawError < 15.0*UnitConversions.DEG_TO_RAD);
              }
              else
              {
                 fail("no detection");
              }
      }
   }
}
