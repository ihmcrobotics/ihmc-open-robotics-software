package us.ihmc.ihmcPerception.linemod;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;

import com.sun.jna.Native;

import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

class LineModInterface
{

   static
   {
      Native.register(NativeLibraryLoader.extractLibrary("us.ihmc.ihmcPerception.linemod", "linemod_interface"));
   }
   

   private static native int trainTemplateBytes (int w, int h, float[] xyzrgb, int[] mask, byte[] outbuf, int outlen, int numberOfPointPerFeatureModality);
   private static native int matchTemplatesBytes(int w, int h, float[] xyzrgb, int nr_templates, byte[]buf, int buflen, int averaging, int nonMaximalSupression);

   

   public static LineModTemplate trainTemplateBytes(OrganizedPointCloud pointCloud, int[] mask)
   {
      byte[] buf = new byte[16384];
      int outlen=trainTemplateBytes(pointCloud.width, pointCloud.height, pointCloud.xyzrgb, mask, buf, buf.length, 300);

      byte[] out=new byte[outlen];
      System.arraycopy(buf, 0, out, 0, outlen);
      LineModTemplate template = new LineModTemplate(out);
      return template;
   }
   public static ArrayList<LineModDetection> matchTemplatesBytes(OrganizedPointCloud testCloud, ArrayList<LineModTemplate> templates, boolean averaging, boolean nonMaximalSupression)
   {
      int totalLength=0;
      for(int i=0;i<templates.size();i++)
         totalLength+=templates.get(i).buf.length;
      byte[] concatenatedTemplates = new byte[totalLength];
      int ptr=0;
      for(int i=0;i<templates.size();i++)
      {
         System.arraycopy(templates.get(i).buf, 0, concatenatedTemplates,  ptr, templates.get(i).buf.length);
         ptr+=templates.get(i).buf.length;
      }
      int numberDetecton=matchTemplatesBytes(testCloud.width, testCloud.height, testCloud.xyzrgb,  templates.size(), concatenatedTemplates, totalLength, averaging?1:0, nonMaximalSupression?1:0);
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


   
 
}
