package us.ihmc.perception.detections.yolo;

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
   private final Map<Integer, Target2D> targets = new LinkedHashMap<>();

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
      Set<Integer> updated = new HashSet<>();

      // 1) assign obs -> targets
      for (Obs o : observations)
      {
         Target2D matched = null;

         // (A) match by trackId if valid
         if (o.trackId >= 0)
         {
            for (Target2D t : targets.values())
            {
               if (t.lastTrackId == o.trackId)
               {
                  matched = t;
                  break;
               }
            }
         }

         // (B) else match by IoU
         if (matched == null)
         {
            float best = 0f;
            for (Target2D t : targets.values())
            {
               float iou = iou(o.bbox, t.latestBbox);
               if (iou > 0.5f && iou > best)
               {
                  best = iou;
                  matched = t;
               }
            }
         }

         if (matched != null)
         {
            matched.update(o.trackId, o.name, o.mask, o.bbox, o.prob, o.texture, historySize);
            updated.add(matched.targetId);
         }
         else
         {
            int id = nextTargetId++;
            Target2D nt = new Target2D(id, o.trackId, o.name, o.mask, o.bbox, o.prob, o.texture, historySize);
            targets.put(id, nt);
            updated.add(id);
         }
      }

      // 2) mark missed + delete stale
      Iterator<Map.Entry<Integer, Target2D>> it = targets.entrySet().iterator();
      while (it.hasNext())
      {
         Target2D t = it.next().getValue();
         if (!updated.contains(t.targetId))
         {
            t.markMissed(historySize);
            if (t.missedFrames > maxMissedFrames)
            {
               it.remove();
               continue;
            }
         }

         t.score = computeScore(t, frameW, frameH);

         if (t.score < scoreStop)
            it.remove();
      }

      // 3) publish good targets
      List<Target2D> published = new ArrayList<>();
      for (Target2D t : targets.values())
      {
         float s = computeScore(t, frameW, frameH);
         if (s >= scoreStart &&
             t.totalSeenFrames() >= minFrameCount &&
             t.missedFrames <= 5)
         {
            published.add(t);
         }
      }
      return published;
   }

   public static class Obs
   {
      public final Object mask;
      public final float[] bbox;   // [x1,y1,x2,y2]
      public final int trackId;
      public final float prob;
      public final String name;
      public final float texture;

      public Obs(Object mask, float[] bbox, int trackId, float prob, String name, float texture)
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
      float x1 = t.latestBbox[0], y1 = t.latestBbox[1], x2 = t.latestBbox[2], y2 = t.latestBbox[3];
      float bboxArea = max(0f, (x2 - x1)) * max(0f, (y2 - y1));
      float frameArea = (float) frameW * (float) frameH;
      float areaRatio = frameArea > 0 ? bboxArea / frameArea : 0f;

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
      if (areaRatio < minAreaRatio) sizeFactor = (minAreaRatio > 0f) ? areaRatio / minAreaRatio : 0f;
      else if (areaRatio > maxAreaRatio) sizeFactor = (1f - maxAreaRatio > 0f) ? max(0f, (1f - areaRatio) / (1f - maxAreaRatio)) : 0f;
      else sizeFactor = 1f;

      float wSum = weights.prob + weights.temporal + weights.texture + weights.border + weights.size;

      return (weights.prob * avgProb +
              weights.temporal * temporal +
              weights.texture * normTexture +
              weights.border * borderFactor +
              weights.size * sizeFactor) / max(1e-6f, wSum);
   }

   private static float iou(float[] a, float[] b)
   {
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
      float s = 0f; int n = 0;
      for (Float v : xs) { s += v; n++; }
      return n == 0 ? 0f : s / n;
   }

   private static float meanInt(Iterable<Integer> xs)
   {
      float s = 0f; int n = 0;
      for (Integer v : xs) { s += v; n++; }
      return n == 0 ? 0f : s / n;
   }

   private static float clamp01(float v) { return max(0f, min(1f, v)); }
}