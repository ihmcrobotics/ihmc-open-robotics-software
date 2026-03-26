package us.ihmc.perception.detections.yolo;

import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_core.Size;
import perception_msgs.msg.dds.YOLOv8AnnotationRecordMessage;
import std_msgs.msg.dds.MultiArrayDimension;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.idl.IDLSequence;
import us.ihmc.perception.RawImage;

/**
 * Representation of a single YOLOv8 detection for annotation.
 * Does not contain any images to avoid high bandwidth usage
 * when sending annotation information over a network.
 *
 * @param objectClass  The object class of the detection.
 * @param confidence   The confidence of the detection.
 * @param boundingBox  2D bounding box of the detection.
 * @param maskPolygons Array of polygons representing the mask of the detection.
 *                     Each polygon is an integer array containing [x1, y1, x2, y2, ...].
 */
public record YOLOv8AnnotationRecord(String objectClass, float confidence, BoundingBox2DReadOnly boundingBox, int[][] maskPolygons)
{
   private static final int FONT = opencv_imgproc.FONT_HERSHEY_DUPLEX;
   private static final int FONT_THICKNESS = 2;
   private static final double FONT_SCALE = 1.5;
   private static final int LINE_TYPE = opencv_imgproc.LINE_4;
   private static final int TEXT_LINE_TYPE = opencv_imgproc.LINE_AA;

   private static final Scalar GREEN = new Scalar(0.0, 255.0, 0.0, 255.0).retainReference();
   private static final Scalar WHITE = new Scalar(255.0, 255.0, 255.0, 255.0).retainReference();

   /**
    * Draws the mask of the detection on the input image.
    *
    * @param inputImage  Input image. Not modified.
    * @param outputImage Output image containing the mask. Modified.
    * @param fill        Whether to fill the mask or draw an outline.
    * @param opacity     Opacity of the mask. Should be in the range [0.0, 1.0].
    */
   public void drawMask(Mat inputImage, Mat outputImage, boolean fill, double opacity)
   {
      inputImage.copyTo(outputImage);

      // Translate the polygons from int[][] to MatVector
      int[][] polygons = maskPolygons();
      MatVector polygonsVector = new MatVector(polygons.length);
      for (int i = 0; i < polygons.length; ++i)
      {
         int[] polygon = polygons[i];
         Mat polygonMat = new Mat(polygon.length / 2, 1, opencv_core.CV_32SC2, new IntPointer(polygon));
         polygonsVector.put(i, polygonMat);
      }

      // Create an overlay image and draw the mask onto the overlay
      Mat overlay = new Mat();
      outputImage.copyTo(overlay);

      if (fill)
         opencv_imgproc.fillPoly(overlay, polygonsVector, GREEN);
      else
         opencv_imgproc.polylines(overlay, polygonsVector, true, GREEN, 5, LINE_TYPE, 0);

      // Blend the overlay onto the output image with the specified opacity
      opacity = MathTools.clamp(opacity, 0.0, 1.0);
      if (opacity >= 1.0)
         overlay.copyTo(outputImage);
      else if (opacity > 0.0)
         opencv_core.addWeighted(overlay, opacity, outputImage, 1.0 - opacity, 0.0, outputImage);

      overlay.close();
      for (int i = 0; i < polygons.length; ++i)
         polygonsVector.get(i).close();
      polygonsVector.close();
   }

   /**
    * Draws the bounding box of the detection on the input image.
    *
    * @param inputImage  Input image. Not modified.
    * @param outputImage Output image containing the bounding box. Modified.
    */
   public void drawBoundingBox(Mat inputImage, Mat outputImage)
   {
      inputImage.copyTo(outputImage);

      Rect boundingBoxRect = new Rect((int) Math.round(boundingBox.getMinX()),
                                      (int) Math.round(boundingBox.getMinY()),
                                      (int) Math.round(boundingBox.getMaxX() - boundingBox.getMinX()),
                                      (int) Math.round(boundingBox.getMaxY() - boundingBox.getMinY()));
      opencv_imgproc.rectangle(outputImage, boundingBoxRect, GREEN, 5, LINE_TYPE, 0);
   }

   /**
    * Draw text containing the object class and confidence on the input image.
    *
    * @param inputImage  Input image. Not modified.
    * @param outputImage Output image containing the text. Modified.
    * @param drawTextBox Whether to put the text in a box.
    * @param inlay       Whether to attempt to inlay the text inside the detection.
    */
   public void drawText(Mat inputImage, Mat outputImage, boolean drawTextBox, boolean inlay)
   {
      inputImage.copyTo(outputImage);

      // Get the text and its size
      String text = String.format("%s: %.2f", objectClass, confidence);
      Size textSize = opencv_imgproc.getTextSize(text, FONT, FONT_SCALE, FONT_THICKNESS, new IntPointer());

      // Determine the text position
      int textX;
      int textY;

      if (inlay)
      {
         // If inlaying, put the text in the center of the detection unless the text is wider than the detected object
         Point2D centerPoint = new Point2D();
         boundingBox.getCenterPoint(centerPoint);
         textX = (int) Math.round(centerPoint.getX()) - textSize.width() / 2;
         if (textSize.width() >= boundingBox.getMaxX() - boundingBox.getMinX())
            textY = (int) Math.round(boundingBox.getMinY()) - textSize.height();
         else
            textY = (int) Math.round(centerPoint.getY()) - textSize.height() / 2;
      }
      else
      {
         // Otherwise, put the text at the top left corner of the detection
         textX = (int) Math.round(boundingBox.getMinX());
         textY = (int) Math.round(boundingBox.getMinY()) - textSize.height();
      }

      // Clamp to ensure the text is within the image bounds
      int textBoxClampedX = MathTools.clamp(textX, 0, outputImage.cols() - textSize.width());
      int textBoxClampedY = MathTools.clamp(textY, 0, outputImage.rows() - textSize.height());

      // Optionally draw the text background
      if (drawTextBox)
      {
         Rect textBox = new Rect(textBoxClampedX, textBoxClampedY, textSize.width(), textSize.height());
         opencv_imgproc.rectangle(outputImage, textBox, GREEN, opencv_imgproc.FILLED, LINE_TYPE, 0);
      }

      // Draw the text
      Scalar textColor = drawTextBox ? WHITE : GREEN;
      Point textLocation = new Point(textBoxClampedX, textBoxClampedY + textSize.height());
      opencv_imgproc.putText(outputImage, text, textLocation, FONT, FONT_SCALE, textColor, FONT_THICKNESS, TEXT_LINE_TYPE, false);
   }

   public void toMessage(YOLOv8AnnotationRecordMessage message)
   {
      message.setObjectClass(objectClass);
      message.setConfidence(confidence);

      Point2D centerPoint = new Point2D();
      boundingBox.getCenterPoint(centerPoint);
      message.getBoundingBox().getCenter().getPosition().setX(centerPoint.getX());
      message.getBoundingBox().getCenter().getPosition().setY(centerPoint.getY());
      message.getBoundingBox().setSizeX(boundingBox.getMaxX() - boundingBox.getMinX());
      message.getBoundingBox().setSizeY(boundingBox.getMaxY() - boundingBox.getMinY());

      message.getMaskPolygons().getData().clear();
      message.getMaskPolygons().getLayout().getDim().clear();
      for (int i = 0; i < maskPolygons.length; ++i)
      {
         int[] polygon = maskPolygons[i];
         MultiArrayDimension dimension = message.getMaskPolygons().getLayout().getDim().add();
         dimension.setLabel("polygon " + i);
         dimension.setSize(polygon.length);
         dimension.setStride((long) Integer.BYTES * polygon.length);

         message.getMaskPolygons().getData().add(polygon);
      }
      message.getMaskPolygons().getLayout().setDataOffset(0);
   }

   public static YOLOv8AnnotationRecord fromMessage(YOLOv8AnnotationRecordMessage message)
   {
      double halfSizeX = 0.5 * message.getBoundingBox().getSizeX();
      double halfSizeY = 0.5 * message.getBoundingBox().getSizeY();

      double minX = message.getBoundingBox().getCenter().getPosition().getX() - halfSizeX;
      double minY = message.getBoundingBox().getCenter().getPosition().getY() - halfSizeY;
      double maxX = message.getBoundingBox().getCenter().getPosition().getX() + halfSizeX;
      double maxY = message.getBoundingBox().getCenter().getPosition().getY() + halfSizeY;
      BoundingBox2D boundingBox = new BoundingBox2D(minX, minY, maxX, maxY);

      IDLSequence.Object<MultiArrayDimension> dimensions = message.getMaskPolygons().getLayout().getDim();
      int[][] maskPolygons = new int[dimensions.size()][];
      for (int i = 0, offset = (int) message.getMaskPolygons().getLayout().getDataOffset() / Integer.BYTES; i < dimensions.size(); ++i)
      {
         maskPolygons[i] = new int[(int) dimensions.get(i).getSize()];
         message.getMaskPolygons().getData().toArray(maskPolygons[i], offset, maskPolygons[i].length);
         offset += maskPolygons[i].length;
      }

      return new YOLOv8AnnotationRecord(message.getObjectClassAsString(), message.getConfidence(), boundingBox, maskPolygons);
   }

   public static YOLOv8AnnotationRecord fromYOLOv8Detection(YOLOv8Detection detection, Size detectionImageSize, float precision)
   {
      RawImage maskImage = detection.mask();
      Mat mask = maskImage.getCpuImageMat();

      Mat resizedMask = new Mat();
      YOLOv8Tools.resizeWithCrop(mask, resizedMask, detectionImageSize);

      YOLOv8AnnotationRecord record = new YOLOv8AnnotationRecord(detection.objectClass(),
                                                                 detection.confidence(),
                                                                 detection.boundingBox(),
                                                                 YOLOv8Tools.getMaskAsPolygons(resizedMask, precision));

      resizedMask.close();
      maskImage.release();

      return record;
   }

   public static YOLOv8AnnotationRecord fromYOLOv8InstantDetection(YOLOv8InstantDetection detection, float precision)
   {
      RawImage maskImage = detection.getObjectMask().get();
      RawImage colorImage = detection.getColorImage().get();
      Mat mask = maskImage.getCpuImageMat();

      Mat resizedMask = new Mat();
      YOLOv8Tools.resizeWithCrop(mask, resizedMask, colorImage.getCpuImageMat().size());

      YOLOv8AnnotationRecord record = new YOLOv8AnnotationRecord(detection.getDetectedObjectClass(),
                                                                 (float) detection.getConfidence(),
                                                                 detection.getBoundingBox(),
                                                                 YOLOv8Tools.getMaskAsPolygons(resizedMask, precision));

      resizedMask.close();
      colorImage.release();
      maskImage.release();

      return record;
   }
}
