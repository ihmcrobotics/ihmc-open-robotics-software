package us.ihmc.ihmcPerception.linemod;

import java.io.File;
import java.util.ArrayList;

import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.sun.jna.Memory;
import com.sun.jna.Pointer;

public class LineModDetector
{
   
   String modelPathInResource;
   Pointer[] signature;
   ArrayList<byte[]>  byteFeatures = new ArrayList<>(); 

   public LineModDetector(String modelPathInResource)
   {
      this.modelPathInResource = modelPathInResource;

   }

   
   private OrganizedPointCloud renderCloud(float yawAngle)
   {
      Render3dObject renderer = new Render3dObject(new File(modelPathInResource));

      //
      final Quaternion qX = new Quaternion();
      qX.fromAngles(FastMath.PI / 2, 0.0f, 0.0f);
      final Quaternion qYaw = new Quaternion();
      qYaw.fromAngles(0, 0,yawAngle);
      final Quaternion qTilt = new Quaternion();
      qTilt.fromAngles(FastMath.PI / 4, 0.0f, 0.0f);
      Transform transform = new Transform(qTilt.mult(qYaw.mult(qX)));
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
//         mask[i]=(char) ((i%2==0)?1:0);
      }
      return mask;
   }
   
   public ArrayList<LineModDetection> detectObjectAndEstimatePose()
   {
      OrganizedPointCloud testCloud = renderCloud((float)Math.PI/12);
//      return LineModInterface.matchTemplates(testCloud, signature, true);
      return LineModInterface.matchTemplatesBytes(testCloud, byteFeatures);
   }

   public void trainModelFromRenderedImage()
   {
      this.signature=new Pointer[2];
      for(int i=0;i<signature.length;i++)
      {
         OrganizedPointCloud pointCloud = renderCloud((float)Math.PI*2*i/signature.length);
         int[] mask = makeMaskByZThresholing(pointCloud, 1.5f);
         this.signature[i]=LineModInterface.trainTemplate(pointCloud, mask);
         byte[] data=LineModInterface.trainTemplateBytes(pointCloud, mask);
         byteFeatures.add(data);
      }
   }
   
   
   public static void main(String[] arg)
   {
      LineModDetector detector = new LineModDetector("/examples/drill/drill.obj");
      detector.trainModelFromRenderedImage();
      ArrayList<LineModDetection> detections  = detector.detectObjectAndEstimatePose();
      for(LineModDetection detection :detections)
         System.out.println(detection);

   }
}
