package us.ihmc.perception.detections.yolo;

import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Rect;
import us.ihmc.perception.RawImage;

import java.util.ArrayDeque;
import java.util.Deque;

import static org.bytedeco.opencv.global.opencv_core.CV_32F;
import static org.bytedeco.opencv.global.opencv_core.NORM_MINMAX;
import static org.bytedeco.opencv.global.opencv_core.add;
import static org.bytedeco.opencv.global.opencv_core.countNonZero;
import static org.bytedeco.opencv.global.opencv_core.multiply;
import static org.bytedeco.opencv.global.opencv_core.normalize;
import static org.bytedeco.opencv.global.opencv_core.sqrt;
import static org.bytedeco.opencv.global.opencv_imgproc.threshold;
import static org.bytedeco.opencv.global.opencv_imgproc.COLOR_BGR2GRAY;
import static org.bytedeco.opencv.global.opencv_imgproc.GaussianBlur;
import static org.bytedeco.opencv.global.opencv_imgproc.Sobel;
import static org.bytedeco.opencv.global.opencv_imgproc.cvtColor;
import static org.bytedeco.opencv.global.opencv_imgproc.THRESH_BINARY;

class Target2D
{
   public final int targetId;
   public int lastTrackId;
   public String name;

   public RawImage latestMask;
   public float[] latestBbox;

   public int missedFrames = 0;
   public final Deque<Integer> frameCount;
   public final Deque<Float> probHistory;
   public final Deque<Float> textureHistory;

   public float score = 0.0f;

   public Target2D(int targetId,
                   int initialTrackId,
                   String name,
                   RawImage mask,
                   float[] bbox,
                   float prob,
                   float texture,
                   int historySize)
   {
      this.targetId = targetId;
      this.lastTrackId = initialTrackId;
      this.name = name;

      this.latestMask = (mask == null) ? null : mask.get();
      this.latestBbox = (bbox == null) ? null : bbox.clone();

      this.frameCount = new ArrayDeque<>(historySize);
      this.probHistory = new ArrayDeque<>(historySize);
      this.textureHistory = new ArrayDeque<>(historySize);

      frameCount.addLast(1);
      probHistory.addLast(prob);
      textureHistory.addLast(texture);
   }

   public void update(int trackId,
                      String name,
                      RawImage mask,
                      float[] bbox,
                      float prob,
                      float texture,
                      int historySize)
   {
      this.lastTrackId = trackId;
      this.name = name;
      this.latestBbox = (bbox == null) ? null : bbox.clone();

      RawImage newMaskRef = (mask == null) ? null : mask.get();
      if (this.latestMask != null)
         this.latestMask.release();
      this.latestMask = newMaskRef;

      pushCapped(frameCount, 1, historySize);
      pushCapped(probHistory, prob, historySize);
      pushCapped(textureHistory, texture, historySize);

      missedFrames = 0;
   }

   public void markMissed(int historySize)
   {
      missedFrames++;
      pushCapped(frameCount, 0, historySize);
   }

   public int totalSeenFrames()
   {
      int s = 0;
      for (int v : frameCount) s += v;
      return s;
   }

   public void destroy()
   {
      if (latestMask != null)
      {
         latestMask.release();
         latestMask = null;
      }
      latestBbox = null;
   }

   private static <T> void pushCapped(Deque<T> q, T v, int cap)
   {
      if (q.size() >= cap)
         q.removeFirst();
      q.addLast(v);
   }
}

class AnnotatedTarget2D
{
   public final int targetId;
   public final int trackId;
   public final String name;
   public final float score;
   public final float[] bbox;
   public final RawImage mask;

   public AnnotatedTarget2D(int targetId, int trackId, String name, float score, float[] bbox, RawImage mask)
   {
      this.targetId = targetId;
      this.trackId = trackId;
      this.name = name;
      this.score = score;
      this.bbox = bbox;
      this.mask = mask;
   }

   public void destroy()
   {
      if (mask != null)
         mask.release();
   }
}

class TextureTools
{
   private TextureTools() {}

   public static Mat computeTextureMap01(Mat bgr)
   {
      if (bgr == null || bgr.isNull())
         return null;

      Mat gray = new Mat();
      cvtColor(bgr, gray, COLOR_BGR2GRAY);

      Mat grayBlur = new Mat();
      GaussianBlur(gray, grayBlur, new org.bytedeco.opencv.opencv_core.Size(3, 3), 0);

      Mat gradX = new Mat();
      Mat gradY = new Mat();
      Sobel(grayBlur, gradX, CV_32F, 1, 0);
      Sobel(grayBlur, gradY, CV_32F, 0, 1);

      Mat gradX2 = new Mat();
      Mat gradY2 = new Mat();
      multiply(gradX, gradX, gradX2);
      multiply(gradY, gradY, gradY2);

      Mat mag2 = new Mat();
      add(gradX2, gradY2, mag2);

      Mat magnitude = new Mat();
      sqrt(mag2, magnitude);

      Mat tex = new Mat();
      GaussianBlur(magnitude, tex, new org.bytedeco.opencv.opencv_core.Size(15, 15), 0);

      Mat tex01 = new Mat();
      normalize(tex, tex01, 0.0, 1.0, NORM_MINMAX, CV_32F, null);

      gray.release();
      grayBlur.release();
      gradX.release();
      gradY.release();
      gradX2.release();
      gradY2.release();
      mag2.release();
      magnitude.release();
      tex.release();

      return tex01;
   }

   public static float meanTextureInMask(Mat textureMap01, Mat mask, float[] bbox)
   {
      if (textureMap01 == null || textureMap01.isNull() || mask == null || mask.isNull() || bbox == null || bbox.length < 4)
         return 0f;

      int W = Math.min(textureMap01.cols(), mask.cols());
      int H = Math.min(textureMap01.rows(), mask.rows());
      if (W <= 1 || H <= 1)
         return 0f;

      int x1 = Math.round(bbox[0]);
      int y1 = Math.round(bbox[1]);
      int x2 = Math.round(bbox[2]);
      int y2 = Math.round(bbox[3]);

      if (x2 < x1) { int tmp = x1; x1 = x2; x2 = tmp; }
      if (y2 < y1) { int tmp = y1; y1 = y2; y2 = tmp; }

      x1 = clamp(x1, 0, W - 1);
      y1 = clamp(y1, 0, H - 1);
      x2 = clamp(x2, 0, W);
      y2 = clamp(y2, 0, H);

      int rw = Math.max(1, x2 - x1);
      int rh = Math.max(1, y2 - y1);

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
      org.bytedeco.opencv.global.opencv_core.meanStdDev(texRoi, mean, std, maskBin);

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
}