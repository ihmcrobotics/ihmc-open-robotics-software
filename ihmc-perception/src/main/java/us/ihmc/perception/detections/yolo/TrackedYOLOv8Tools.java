package us.ihmc.perception.detections.yolo;

import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.commons.MathTools;

import java.util.List;
import java.util.Locale;

/** Drawing utilities used only by {@link TrackedYOLOv8DetectionExecutor}. */
final class TrackedYOLOv8Tools
{
   private static final int LINE_TYPE = opencv_imgproc.LINE_8;
   private static final int TEXT_LINE_TYPE = opencv_imgproc.LINE_AA;
   private static final int FONT = opencv_imgproc.FONT_HERSHEY_SIMPLEX;
   private static final double FONT_SCALE = 1.0;
   private static final int FONT_THICKNESS = 2;
   private static final Scalar WHITE = new Scalar(255.0, 255.0, 255.0, 255.0);

   private TrackedYOLOv8Tools()
   {
   }

   static void annotateTargets(Mat image, List<AnnotatedTarget2D> targets)
   {
      for (AnnotatedTarget2D target : targets)
      {
         if (target.bbox == null)
            continue;

         int x1 = Math.round(target.bbox[0]);
         int y1 = Math.round(target.bbox[1]);
         int x2 = Math.round(target.bbox[2]);
         int y2 = Math.round(target.bbox[3]);
         Scalar color = colorForId(target.targetId);

         if (target.mask != null)
         {
            Mat mask = target.mask.getCpuImageMat();
            if (mask != null && !mask.isNull())
            {
               Mat resizedMask = new Mat();
               YOLOv8Tools.resizeWithCrop(mask, resizedMask, image.size());
               Mat colorMat = new Mat(image.rows(), image.cols(), image.type(), color);
               opencv_core.add(image, colorMat, image, resizedMask, -1);
               colorMat.close();
               resizedMask.close();
            }
         }

         opencv_imgproc.rectangle(image, new Point(x1, y1), new Point(x2, y2), color, 4, LINE_TYPE, 0);

         String objectName = prettifyName(target.name);
         String label = String.format(Locale.US, "ID:%d %.2f %s", target.targetId, target.score, objectName);
         Size textSize = opencv_imgproc.getTextSize(label, FONT, FONT_SCALE, FONT_THICKNESS, new IntPointer());
         int textX = MathTools.clamp(x1, 0, image.cols() - textSize.width());
         int textY = MathTools.clamp(y1 - textSize.height(), 0, image.rows() - textSize.height());
         Rect textBox = new Rect(textX, textY, textSize.width(), textSize.height());
         opencv_imgproc.rectangle(image, textBox, color, opencv_imgproc.FILLED, LINE_TYPE, 0);
         opencv_imgproc.putText(image,
                               label,
                               new Point(textX, textY + textSize.height()),
                               FONT,
                               FONT_SCALE,
                               WHITE,
                               FONT_THICKNESS,
                               TEXT_LINE_TYPE,
                               false);
         textBox.close();
      }
   }

   private static Scalar colorForId(int id)
   {
      int colorId = Math.max(id, 0) + 1;
      return new Scalar((colorId * 97) % 255, (colorId * 17) % 255, (colorId * 37) % 255, 255.0);
   }

   private static String prettifyName(String name)
   {
      if (name == null || name.isBlank())
         return "Object";

      String cleaned = name.replace('_', ' ').trim().toLowerCase(Locale.US);
      return cleaned.isEmpty() ? "Object" : Character.toUpperCase(cleaned.charAt(0)) + cleaned.substring(1);
   }
}
