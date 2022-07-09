package us.ihmc.perception;

import org.bytedeco.opencv.global.opencv_aruco;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_aruco.Dictionary;
import org.bytedeco.opencv.opencv_core.Mat;

public class OpenCVArUcoMarkerGenerator
{
   /**
    * Save a ArUco marker image of id to file.
    *
    * To print a marker:
    *
    * Use GIMP to create a US Letter (ANSI A) 8.5" x 11" document. Use File > New...
    *
    * Set the document to 300 pixels per inch (300 ppi).
    *
    * Size your marker in pixels. 1 centimeter = 0.393701 inches.
    * So say we want a 20 cm marker. 20 * 0.393701 = 7.87402 * 300 = ~2362 pixels.
    *
    * Let's get the exact width from the rouned pixels to meters. 1 pixel = 0.0254 / 300.0.
    * So 2362 pixels is a 0.1999826667 meter wide marker.
    *
    * Set totalImageSizePixels below to 2362 and add as layers to the GIMP image and print.
    * Check with a ruler that it's about 20 cm wide.
    *
    * The above is sometimes too big to fit on the printed area of a page, so:
    * 2000 px = 0.1693333333 meters = 6.66666666535433 inches
    *
    * Everytime I print the 2000 px one I get 16.8 cm wide code.
    *
    * Make sure the Print dialog does not scale the image. You might need to tell it to "Ignore print margins"
    * and manually say to print the image as 8.5" x 11".
    *
    */
   public static void main(String[] args)
   {
      Mat markerToSave = new Mat();
      Dictionary dictionary = opencv_aruco.getPredefinedDictionary(OpenCVArUcoMarkerDetection.DEFAULT_DICTIONARY);
      int markerID = 0;
      int totalImageSizePixels = 2000;
      for (; markerID < 1; markerID++)
      {
         opencv_aruco.drawMarker(dictionary, markerID, totalImageSizePixels, markerToSave, 2);
         opencv_imgcodecs.imwrite("marker" + markerID + ".png", markerToSave);
      }
   }
}
