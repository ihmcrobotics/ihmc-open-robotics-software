package us.ihmc.ihmcPerception.linemod;

import java.awt.Color;
import java.net.URI;
import java.net.URISyntaxException;

import javax.vecmath.Point3d;

import com.sun.jna.Native;
import com.sun.jna.Pointer;

import sensor_msgs.PointCloud2;
import us.ihmc.utilities.nativelibraries.NativeLibraryLoader;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

class LineModInterface
{

   static
   {
      Native.register(NativeLibraryLoader.extractLibrary("us.ihmc.ihmcPerception.linemod", "linemod_interface"));
   }
   
   public static native int test(int a);

   public static native Pointer loadTemplates(String templates_filename, int[] nr_templates);
   public static native Pointer loadTemplate(String templates_filename);
   public static native Pointer trainTemplate(int w, int h, float[] xyzrgb, char[]mask);
   public static native int matchTemplates(int w, int h, float[] xyzrgb, int nr_templates, Pointer templates, int debug);
   
   
   
   public static void testDetection()
   {
      int[] nr_templates = new int[1];
	   Pointer p = loadTemplates("csrc/linemod/test_template.sqmmt", nr_templates);
   }
   
   public static void liveDetection()
   {
      //load templates
      final int[] nr_templates = new int[1];
	   final Pointer templates = loadTemplates("csrc/linemod/test_template.sqmmt", nr_templates);
	   
	   //setup cloud listener
	   RosPointCloudSubscriber rosPointCloudSubscriber  = new RosPointCloudSubscriber()
      {
         
         @Override
         public void onNewMessage(PointCloud2 pointCloud)
         {
            UnpackedPointCloud unpackedPointCloud = unpackPointsAndIntensities(pointCloud);
            int nPoints=pointCloud.getWidth()*pointCloud.getHeight();
            float[] xyzrgb = new float[nPoints*4];

            Point3d[] xyz=unpackedPointCloud.getPoints();
            Color[] rgb=unpackedPointCloud.getPointColors();
            int counter=0;
            for(int i=0;i<nPoints;i++)
            {
               xyzrgb[counter++] = (float) xyz[i].getX();
               xyzrgb[counter++] = (float) xyz[i].getY();
               xyzrgb[counter++] = (float) xyz[i].getZ();
               xyzrgb[counter++] = Float.intBitsToFloat(rgb[i].getRGB());
            }
            int numDetects=matchTemplates(pointCloud.getWidth(), pointCloud.getHeight(),xyzrgb, nr_templates[0], templates, 1);
            System.out.println("detects "+numDetects);

         }
      };
      
      try
      {
         String topic = "/cloud_pcd";
         RosMainNode mainNode = new RosMainNode(new URI("http://localhost:11311/"), "linemod");
         mainNode.attachSubscriber(topic,rosPointCloudSubscriber);
         mainNode.execute();
      }
      catch (URISyntaxException e)
      {
         e.printStackTrace();
      }
      
   }

	public static void main(String[] arg)
	{
	   System.out.println("test:"+test(1));
	   testDetection();
	   liveDetection();
	}

}
