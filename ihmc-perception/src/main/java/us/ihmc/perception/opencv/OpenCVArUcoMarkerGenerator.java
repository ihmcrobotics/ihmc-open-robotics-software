package us.ihmc.perception.opencv;

import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_objdetect;
import org.bytedeco.opencv.opencv_objdetect.Dictionary;
import org.bytedeco.opencv.opencv_core.Mat;

public class OpenCVArUcoMarkerGenerator
{
   /**
    *
    * Updated method:
    *
    * You can also use this site to generate markers as SVG: https://chev.me/arucogen/
    * Use Inkscape to size them and place them onto a canvas for printing.
    * Save as PDF in Inkscape. Use Custom:
    * Left: 0  Top: 0
    * Right: 24 in  Bottom: 30 in
    * Width: 24 in  Height: 30 in
    * Image size:
    * Width: 710 px  Height: 888 px
    * DPI: 29.60
    *
    * ---
    *
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
    * Set totalImageSizePixels below to 2362 and File > Open as Layers to the GIMP image and print.
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
      Dictionary dictionary = opencv_objdetect.getPredefinedDictionary(OpenCVArUcoMarkerDetector.DEFAULT_DICTIONARY);
      int startingMarkerID = 0;
      int numberOfSequentialMarkersToGenerate = 4;
      int endIndex = startingMarkerID + numberOfSequentialMarkersToGenerate;
      int totalImageSizePixels = 2000;
      for (; startingMarkerID < endIndex; startingMarkerID++)
      {
//         opencv_aruco.drawMarker(dictionary, startingMarkerID, totalImageSizePixels, markerToSave, 2);
         opencv_imgcodecs.imwrite("marker" + startingMarkerID + ".png", markerToSave);
      }
   }
}
