package us.ihmc.perception.detections.yolo;

import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Rect;

import static org.bytedeco.opencv.global.opencv_core.*;
import static org.bytedeco.opencv.global.opencv_imgproc.*;

public class TextureTools
{
   private TextureTools() {}

   /**
    * Compute per-frame texture map equivalent to Python:
    * gray -> GaussianBlur(3) -> Sobel x/y -> magnitude -> GaussianBlur(15) -> normalize [0,1].
    *
    * Returns CV_32F Mat in [0,1], same WxH as input.
    */
   public static Mat computeTextureMap01(Mat bgr)
   {
      if (bgr == null || bgr.isNull())
         return null;

      Mat gray = new Mat();
      cvtColor(bgr, gray, COLOR_BGR2GRAY);

      // blur_size=3
      Mat grayBlur = new Mat();
      GaussianBlur(gray, grayBlur, new org.bytedeco.opencv.opencv_core.Size(3, 3), 0);

      // Sobel gradients (CV_32F)
      Mat gradX = new Mat();
      Mat gradY = new Mat();

      Sobel(grayBlur, gradX, CV_32F, 1, 0);
      Sobel(grayBlur, gradY, CV_32F, 0, 1);

      // magnitude = sqrt(grad_x^2 + grad_y^2)
      Mat gradX2 = new Mat();
      Mat gradY2 = new Mat();
      multiply(gradX, gradX, gradX2);
      multiply(gradY, gradY, gradY2);

      Mat mag2 = new Mat();
      add(gradX2, gradY2, mag2);

      Mat magnitude = new Mat();
      sqrt(mag2, magnitude);

      // blur magnitude with (15,15)
      Mat tex = new Mat();
      GaussianBlur(magnitude, tex, new org.bytedeco.opencv.opencv_core.Size(15, 15), 0);

      // normalize to [0,1] with global min/max (like python)
      Mat tex01 = new Mat();
      normalize(tex, tex01, 0.0, 1.0, NORM_MINMAX, CV_32F, null);

      // cleanup temporaries
      gray.release();
      grayBlur.release();
      gradX.release();
      gradY.release();
      gradX2.release();
      gradY2.release();
      mag2.release();
      magnitude.release();
      tex.release();

      return tex01; // caller owns and must release
   }

   /**
    * Mean of textureMap01 inside mask (mask > 0), optionally restrict to bbox ROI.
    * Equivalent to torch.mean(texture_map[mask>0]).
    *
    * textureMap01 must be CV_32F in [0,1].
    * mask should be single-channel (0/255 or 0/1); any >0 is treated as true.
    */
   public static float meanTextureInMask(Mat textureMap01, Mat mask, float[] bbox)
   {
      if (textureMap01 == null || textureMap01.isNull() || mask == null || mask.isNull() || bbox == null || bbox.length < 4)
         return 0f;

      // Use the actual usable bounds (intersection of both mats)
      int W = Math.min(textureMap01.cols(), mask.cols());
      int H = Math.min(textureMap01.rows(), mask.rows());
      if (W <= 1 || H <= 1)
         return 0f;

      int x1 = Math.round(bbox[0]);
      int y1 = Math.round(bbox[1]);
      int x2 = Math.round(bbox[2]);
      int y2 = Math.round(bbox[3]);

      // Fix inverted boxes
      if (x2 < x1) { int tmp = x1; x1 = x2; x2 = tmp; }
      if (y2 < y1) { int tmp = y1; y1 = y2; y2 = tmp; }

      // Clamp to [0, W/H]
      x1 = clamp(x1, 0, W - 1);
      y1 = clamp(y1, 0, H - 1);
      x2 = clamp(x2, 0, W);     // allow x2 == W (exclusive)
      y2 = clamp(y2, 0, H);     // allow y2 == H (exclusive)

      int rw = Math.max(1, x2 - x1);
      int rh = Math.max(1, y2 - y1);

      // Ensure roi fully inside both mats
      if (x1 + rw > W) rw = W - x1;
      if (y1 + rh > H) rh = H - y1;
      if (rw <= 0 || rh <= 0)
         return 0f;

      Rect roi = new Rect(x1, y1, rw, rh);

      Mat texRoi = new Mat(textureMap01, roi);
      Mat maskRoi = new Mat(mask, roi);

      Mat maskBin = new Mat();
      threshold(maskRoi, maskBin, 0.0, 255.0, THRESH_BINARY);

      Mat mean = new Mat();
      Mat std = new Mat();
      meanStdDev(texRoi, mean, std, maskBin);

      double m = mean.ptr(0, 0).getDouble();

      mean.release();
      std.release();
      maskBin.release();
      texRoi.release();
      maskRoi.release();

      return (float) Math.max(0.0, Math.min(1.0, m));
   }

   private static int clamp(int v, int lo, int hi)
   {
      return Math.max(lo, Math.min(hi, v));
   }

   /** Clamp bbox to image bounds and return ROI rect. */
   private static Rect clampRect(int w, int h, float[] bb)
   {
      if (bb == null || bb.length < 4)
         return new Rect(0, 0, 0, 0);

      int x1 = Math.max(0, Math.min(w - 1, Math.round(bb[0])));
      int y1 = Math.max(0, Math.min(h - 1, Math.round(bb[1])));
      int x2 = Math.max(0, Math.min(w - 1, Math.round(bb[2])));
      int y2 = Math.max(0, Math.min(h - 1, Math.round(bb[3])));

      int rw = Math.max(1, x2 - x1);
      int rh = Math.max(1, y2 - y1);
      return new Rect(x1, y1, rw, rh);
   }
}