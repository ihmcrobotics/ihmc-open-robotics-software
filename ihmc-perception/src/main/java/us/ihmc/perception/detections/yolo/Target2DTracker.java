package us.ihmc.perception.detections.yolo;

import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Rect;
import us.ihmc.perception.RawImage;

import java.util.*;

import static java.lang.Math.*;

public class Target2DTracker
{
   public static class Weights
   {
      public float prob = 1f, temporal = 3f, texture = 2f, border = 3f, size = 1f;
   }

   private final int historySize;
   private final float scoreStart;
   private final float scoreStop;
   private final int minFrameCount;
   private final int maxMissedFrames;

   private final float minAreaRatio;
   private final float maxAreaRatio;
   private final float minTexture;
   private final float maxTexture;
   private final int borderSafeDistance;

   private final Weights weights;

   private int nextTargetId = 0;

   /** targetId -> Target2D */
   private final Map<Integer, Target2D> targets = new LinkedHashMap<>();

   /** BoT-SORT trackId -> targetId (fast lookup). Only for trackId >= 0 */
   private final Map<Integer, Integer> trackIdToTargetId = new HashMap<>();

   public Target2DTracker(int historySize,
                          float scoreStart, float scoreStop,
                          int minFrameCount, int maxMissedFrames,
                          float minAreaRatio, float maxAreaRatio,
                          float minTexture, float maxTexture,
                          int borderSafeDistance,
                          Weights weights)
   {
      this.historySize = historySize;
      this.scoreStart = scoreStart;
      this.scoreStop = scoreStop;
      this.minFrameCount = minFrameCount;
      this.maxMissedFrames = maxMissedFrames;
      this.minAreaRatio = minAreaRatio;
      this.maxAreaRatio = maxAreaRatio;
      this.minTexture = minTexture;
      this.maxTexture = maxTexture;
      this.borderSafeDistance = borderSafeDistance;
      this.weights = weights;
   }

   public List<Target2D> update(int frameW, int frameH, List<Obs> observations)
   {
      Set<Integer> updatedTargetIds = new HashSet<>();

      // assign obs -> targets
      for (Obs o : observations)
      {
         Target2D matched = null;

         // match by BoT-SORT trackId if valid
         if (o.trackId >= 0)
         {
            Integer tid = trackIdToTargetId.get(o.trackId);
            if (tid != null)
               matched = targets.get(tid);
         }

         // else match by bbox IoU
         if (matched == null)
         {
            float bestIou = 0f;

            for (Target2D t : targets.values())
            {
               float bboxIou = iou(o.bbox, t.latestBbox);

               // Python behavior: only match if IoU > 0.5
               if (bboxIou > 0.5f && bboxIou > bestIou)
               {
                  bestIou = bboxIou;
                  matched = t;
               }
            }
         }

         if (matched != null)
         {
            // Only adjust trackId mapping if the NEW trackId is valid
            if (o.trackId >= 0)
            {
               if (matched.lastTrackId >= 0 && matched.lastTrackId != o.trackId)
                  trackIdToTargetId.remove(matched.lastTrackId);

               trackIdToTargetId.put(o.trackId, matched.targetId);
            }

            matched.update(o.trackId, o.name, o.mask, o.bbox, o.prob, o.texture, historySize);
            updatedTargetIds.add(matched.targetId);
         }
         else
         {
            int id = nextTargetId++;
            Target2D nt = new Target2D(id, o.trackId, o.name, o.mask, o.bbox, o.prob, o.texture, historySize);
            targets.put(id, nt);
            updatedTargetIds.add(id);

            if (o.trackId >= 0)
               trackIdToTargetId.put(o.trackId, id);
         }
      }

      // mark missed + delete stale / low-score
      Iterator<Map.Entry<Integer, Target2D>> it = targets.entrySet().iterator();
      while (it.hasNext())
      {
         Map.Entry<Integer, Target2D> e = it.next();
         Target2D t = e.getValue();

         if (!updatedTargetIds.contains(t.targetId))
         {
            t.markMissed(historySize);

            if (t.missedFrames > maxMissedFrames)
            {
               removeTarget(it, t);
               continue;
            }
         }

         t.score = computeScore(t, frameW, frameH);

         if (t.score < scoreStop)
            removeTarget(it, t);
      }

      // publish good targets (recompute score)
      List<Target2D> published = new ArrayList<>();
      for (Target2D t : targets.values())
      {
         float s = computeScore(t, frameW, frameH);
         t.score = s;  // keep internal value updated

         if (s >= scoreStart &&
             t.totalSeenFrames() >= minFrameCount &&
             t.missedFrames <= 5)
         {
            published.add(t);
         }
      }

      return published;
   }

   private static float maskIou(RawImage aMask, RawImage bMask,
                                float[] aBbox, float[] bBbox,
                                int frameW, int frameH)
   {
      if (aMask == null || bMask == null)
         return 0f;
      if (aBbox == null || bBbox == null || aBbox.length < 4 || bBbox.length < 4)
         return 0f;

      Mat A = aMask.getCpuImageMat();
      Mat B = bMask.getCpuImageMat();
      if (A == null || B == null || A.isNull() || B.isNull())
         return 0f;

      int x1 = Math.max(Math.round(aBbox[0]), Math.round(bBbox[0]));
      int y1 = Math.max(Math.round(aBbox[1]), Math.round(bBbox[1]));
      int x2 = Math.min(Math.round(aBbox[2]), Math.round(bBbox[2]));
      int y2 = Math.min(Math.round(aBbox[3]), Math.round(bBbox[3]));

      x1 = clamp(x1, 0, frameW - 1);
      y1 = clamp(y1, 0, frameH - 1);
      x2 = clamp(x2, 0, frameW - 1);
      y2 = clamp(y2, 0, frameH - 1);

      int w = Math.max(0, x2 - x1);
      int h = Math.max(0, y2 - y1);
      if (w < 2 || h < 2)
         return 0f;

      Rect roi = new Rect(x1, y1, w, h);

      // bounds check against mask mats (they should be full-res, but be defensive)
      if (roi.x() + roi.width() > A.cols() || roi.y() + roi.height() > A.rows()) return 0f;
      if (roi.x() + roi.width() > B.cols() || roi.y() + roi.height() > B.rows()) return 0f;

      Mat aR = null, bR = null, aBin = null, bBin = null, inter = null, uni = null;
      try
      {
         aR = new Mat(A, roi);
         bR = new Mat(B, roi);

         aBin = new Mat();
         bBin = new Mat();

         org.bytedeco.opencv.global.opencv_imgproc.threshold(aR, aBin, 0.0, 255.0,
                                                             org.bytedeco.opencv.global.opencv_imgproc.THRESH_BINARY);
         org.bytedeco.opencv.global.opencv_imgproc.threshold(bR, bBin, 0.0, 255.0,
                                                             org.bytedeco.opencv.global.opencv_imgproc.THRESH_BINARY);

         inter = new Mat();
         uni = new Mat();

         org.bytedeco.opencv.global.opencv_core.bitwise_and(aBin, bBin, inter);
         org.bytedeco.opencv.global.opencv_core.bitwise_or(aBin, bBin, uni);

         int interCount = org.bytedeco.opencv.global.opencv_core.countNonZero(inter);
         int unionCount = org.bytedeco.opencv.global.opencv_core.countNonZero(uni);

         if (unionCount <= 0)
            return 0f;

         return (float) interCount / (float) unionCount;
      }
      finally
      {
         if (inter != null) inter.release();
         if (uni != null) uni.release();
         if (aBin != null) aBin.release();
         if (bBin != null) bBin.release();
         if (aR != null) aR.release();
         if (bR != null) bR.release();
      }
   }

   private static int clamp(int v, int lo, int hi)
   {
      return Math.max(lo, Math.min(hi, v));
   }

   private void removeTarget(Iterator<Map.Entry<Integer, Target2D>> it, Target2D t)
   {
      if (t.lastTrackId >= 0)
         trackIdToTargetId.remove(t.lastTrackId);

      t.destroy();
      it.remove();
   }

   public static class Obs
   {
      public final RawImage mask;     // may be null; Target2D retains via mask.get()
      public final float[] bbox;      // [x1,y1,x2,y2]
      public final int trackId;       // BoT-SORT track id (or -1)
      public final float prob;
      public final String name;
      public final float texture;

      public Obs(RawImage mask, float[] bbox, int trackId, float prob, String name, float texture)
      {
         this.mask = mask;
         this.bbox = bbox;
         this.trackId = trackId;
         this.prob = prob;
         this.name = name;
         this.texture = texture;
      }
   }

   private float computeScore(Target2D t, int frameW, int frameH)
   {
      if (t.latestBbox == null || t.latestBbox.length < 4 || frameW <= 0 || frameH <= 0)
         return 0f;

      float x1 = t.latestBbox[0], y1 = t.latestBbox[1], x2 = t.latestBbox[2], y2 = t.latestBbox[3];
      float bboxArea = max(0f, (x2 - x1)) * max(0f, (y2 - y1));
      float frameArea = (float) frameW * (float) frameH;
      float areaRatio = bboxArea / frameArea;

      float avgProb = mean(t.probHistory);
      float temporal = meanInt(t.frameCount);

      float avgTexture = mean(t.textureHistory);
      float normTexture = (maxTexture == minTexture) ? avgTexture : (avgTexture - minTexture) / (maxTexture - minTexture);
      normTexture = clamp01(normTexture);

      float leftDist = x1;
      float topDist  = y1;
      float rightDist = frameW - x2;
      float minBorderDist = min(leftDist, min(topDist, rightDist));
      float borderFactor = clamp01(minBorderDist / (float) borderSafeDistance);

      float sizeFactor;
      if (areaRatio < minAreaRatio)
         sizeFactor = (minAreaRatio > 0f) ? areaRatio / minAreaRatio : 0f;
      else if (areaRatio > maxAreaRatio)
         sizeFactor = (1f - maxAreaRatio > 0f) ? max(0f, (1f - areaRatio) / (1f - maxAreaRatio)) : 0f;
      else
         sizeFactor = 1f;

      float wSum = weights.prob + weights.temporal + weights.texture + weights.border + weights.size;

      return (weights.prob * avgProb +
              weights.temporal * temporal +
              weights.texture * normTexture +
              weights.border * borderFactor +
              weights.size * sizeFactor) / max(1e-6f, wSum);
   }

   private static float iou(float[] a, float[] b)
   {
      if (a == null || b == null || a.length < 4 || b.length < 4)
         return 0f;

      float ix1 = max(a[0], b[0]);
      float iy1 = max(a[1], b[1]);
      float ix2 = min(a[2], b[2]);
      float iy2 = min(a[3], b[3]);

      float iw = max(0f, ix2 - ix1);
      float ih = max(0f, iy2 - iy1);
      float inter = iw * ih;

      float areaA = max(0f, a[2] - a[0]) * max(0f, a[3] - a[1]);
      float areaB = max(0f, b[2] - b[0]) * max(0f, b[3] - b[1]);
      float union = areaA + areaB - inter;

      return union <= 0f ? 0f : inter / union;
   }

   private static float mean(Iterable<Float> xs)
   {
      float s = 0f;
      int n = 0;
      for (Float v : xs)
      {
         s += v;
         n++;
      }
      return n == 0 ? 0f : s / n;
   }

   private static float meanInt(Iterable<Integer> xs)
   {
      float s = 0f;
      int n = 0;
      for (Integer v : xs)
      {
         s += v;
         n++;
      }
      return n == 0 ? 0f : s / n;
   }

   private static float clamp01(float v)
   {
      return max(0f, min(1f, v));
   }

   public void destroy()
   {
      for (Target2D t : targets.values())
         t.destroy();
      targets.clear();
      trackIdToTargetId.clear();
   }
}