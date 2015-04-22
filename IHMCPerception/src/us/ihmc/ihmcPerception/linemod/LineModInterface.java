package us.ihmc.ihmcPerception.linemod;

import java.awt.Color;
import java.net.URI;
import java.net.URISyntaxException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;

import javax.vecmath.Point3d;

import org.apache.poi.ss.formula.ptg.MemAreaPtg;

import com.badlogic.gdx.utils.Array;
import com.sun.jna.Memory;
import com.sun.jna.Native;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.PointerByReference;

import sensor_msgs.PointCloud2;
import us.ihmc.utilities.ArrayTools;
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
   public static native Pointer trainTemplate(int w, int h, float[] xyzrgb, int[]mask);
   public static native Pointer trainTemplates(int w, int h, float[] xyzrgb, int[]mask);
   private static native int matchTemplates(int w, int h, float[] xyzrgb, int nr_templates, Pointer templates, int debug);
   private static native int trainTemplatBytes (int w, int h, float[] xyzrgb, int[] mask, byte[] outbuf, int outlen);
   private static native int matchTemplatesBytes(int w, int h, float[] xyzrgb, int nr_templates, byte[]buf, int buflen);

   
   @Deprecated
   public static Pointer trainTemplates(OrganizedPointCloud pointCloud, int[] mask)
   {
      return trainTemplates(pointCloud.width, pointCloud.height, pointCloud.xyzrgb, mask);
   }
   
   public static Pointer trainTemplate(OrganizedPointCloud pointCloud, int[] mask)
   {
      return trainTemplate(pointCloud.width, pointCloud.height, pointCloud.xyzrgb, mask);
   }

   public static byte[] trainTemplateBytes(OrganizedPointCloud pointCloud, int[] mask)
   {
      byte[] buf = new byte[8192];
      int outlen=trainTemplatBytes(pointCloud.width, pointCloud.height, pointCloud.xyzrgb, mask, buf, buf.length);

      byte[] out=new byte[outlen];
      System.arraycopy(buf, 0, out, 0, outlen);
      return out;
   }

   public static ArrayList<LineModDetection> matchTemplatesBytes(OrganizedPointCloud testCloud, ArrayList<byte[]> templates)
   {
      int totalLength=0;
      for(int i=0;i<templates.size();i++)
         totalLength+=templates.get(i).length;
      byte[] concatenatedTemplates = new byte[totalLength];
      int ptr=0;
      for(int i=0;i<templates.size();i++)
      {
         System.arraycopy(templates.get(i), 0, concatenatedTemplates,  ptr, templates.get(i).length);
         ptr+=templates.get(i).length;
      }
      int numberDetecton=matchTemplatesBytes(testCloud.width, testCloud.height, testCloud.xyzrgb,  templates.size(), concatenatedTemplates, totalLength);
      if(numberDetecton<0)
      {
         throw new RuntimeException("insufficient return buffer");
      }
      else
      {
         ArrayList<LineModDetection> detections = new ArrayList<>();
         ByteBuffer buf=ByteBuffer.wrap(concatenatedTemplates);
         buf.order(ByteOrder.LITTLE_ENDIAN);
         for(int i=0;i<numberDetecton;i++)
         {
            LineModDetection detection = new LineModDetection();
            detection.x = buf.getInt();
            detection.y = buf.getInt();
            detection.template_id = buf.getInt();
            detection.score = buf.getFloat();
            detection.scale = buf.getFloat();
            detections.add(detection);
         }
         return detections;
      }
   }

   public static int matchTemplates(OrganizedPointCloud testCloud, Pointer[] templates, boolean debug)
   {
      Memory packedTemplates = new Memory(Pointer.SIZE*templates.length);
      for(int i=0;i<templates.length;i++)
      {
         packedTemplates.setPointer(Pointer.SIZE*i, templates[i]);
      }
      return matchTemplates(testCloud.width, testCloud.height, testCloud.xyzrgb,  templates.length, packedTemplates, debug?1:0);
   }
   
   public static void testDetection()
   {
      int[] nr_templates = new int[1];
	   Pointer p = loadTemplates("csrc/linemod/test_template.sqmmt", nr_templates);
   }
   
   public static void liveDetection()
   {
      //load templates
      final int[] nr_templates = new int[1];
	   final Pointer templates;
	   templates= loadTemplates("csrc/linemod/test_template.sqmmt", nr_templates);
	   
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
         URI rosMasterUri = new URI("http://localhost:11311/");
         String topic = "/cloud_pcd";
         RosMainNode mainNode = new RosMainNode(rosMasterUri, "linemod");
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
