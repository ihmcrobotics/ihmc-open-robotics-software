package us.ihmc.perception;

import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_stitching.Stitcher;

import static org.bytedeco.opencv.global.opencv_imgcodecs.imread;
import static org.bytedeco.opencv.global.opencv_imgcodecs.imwrite;
import static org.bytedeco.opencv.global.opencv_stitching.createStitcher;

public class BytedecoOpenCVDemo
{
   static boolean try_use_gpu = false;
   static MatVector imgs = new MatVector();
   static String result_name = "result.jpg";

   public static void main(String[] args) {
      /* try to use MKL when available */
      System.setProperty("org.bytedeco.openblas.load", "mkl");

      int retval = parseCmdArgs(args);
      if (retval != 0) {
         System.exit(-1);
      }

      Mat pano = new Mat();
      Stitcher stitcher = createStitcher(try_use_gpu);
      int status = stitcher.stitch(imgs, pano);

      if (status != Stitcher.OK) {
         System.out.println("Can't stitch images, error code = " + status);
         System.exit(-1);
      }

      imwrite(result_name, pano);
      System.exit(0);
   }

   static void printUsage() {
      System.out.println(
            "Rotation model images stitcher.\n\n"
            + "stitching img1 img2 [...imgN]\n\n"
            + "Flags:\n"
            + "  --try_use_gpu (yes|no)\n"
            + "      Try to use GPU. The default value is 'no'. All default values\n"
            + "      are for CPU mode.\n"
            + "  --output <result_img>\n"
            + "      The default is 'result.jpg'.");
   }

   static int parseCmdArgs(String[] args) {
      if (args.length == 0) {
         printUsage();
         return -1;
      }
      for (int i = 0; i < args.length; i++) {
         if (args[i].equals("--help") || args.equals("/?")) {
            printUsage();
            return -1;
         } else if (args[i].equals("--try_use_gpu")) {
            if (args[i + 1].equals("no")) {
               try_use_gpu = false;
            } else if (args[i + 1].equals("yes")) {
               try_use_gpu = true;
            } else {
               System.out.println("Bad --try_use_gpu flag value");
               return -1;
            }
            i++;
         } else if (args[i].equals("--output")) {
            result_name = args[i + 1];
            i++;
         } else {
            Mat img = imread(args[i]);
            if (img.empty()) {
               System.out.println("Can't read image '" + args[i] + "'");
               return -1;
            }
            imgs.resize(imgs.size() + 1);
            imgs.put(imgs.size() - 1, img);
         }
      }
      return 0;
   }
}
