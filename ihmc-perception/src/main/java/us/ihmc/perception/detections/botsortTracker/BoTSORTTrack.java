package us.ihmc.perception.detections.botsortTracker;

import java.util.ArrayDeque;

public final class BoTSORTTrack {
   private final int targetId;
   private final int historySize;

   private String name;
   private float[] bbox;     // {x1,y1,x2,y2}
   private int detTrackId;   // detector-provided id if any, else -1
   private int missedFrames = 0;
   private float score = 0f;

   private final ArrayDeque<Float>   probHist   = new ArrayDeque<>();
   private final ArrayDeque<Float>   texHist    = new ArrayDeque<>();
   private final ArrayDeque<Integer> frameCount = new ArrayDeque<>();

   public BoTSORTTrack(int targetId, float[] bbox, int detTrackId, float prob, String name, float texture, int historySize) {
      this.targetId = targetId;
      this.historySize = historySize;
      update(bbox, detTrackId, prob, name, texture); // seeds histories once
   }

   public void update(float[] bbox, int detTrackId, float prob, String name, float texture) {
      this.bbox = bbox;
      this.detTrackId = detTrackId;
      this.name = name;
      pushBounded(probHist, prob);
      pushBounded(texHist, texture);
      pushBounded(frameCount, 1);
      missedFrames = 0;
   }

   public void markMissed() {
      missedFrames++;
      pushBounded(frameCount, 0);
   }

   public int framesSeen() {
      int sum = 0; for (int v : frameCount) sum += v; return sum;
   }

   public float computeScore(
         int frameW, int frameH,
         float minAreaRatio, float maxAreaRatio,
         float texMin, float texMax,
         float borderSafeDistance,
         float wProb, float wTemporal, float wTexture, float wBorder, float wSize) {

      final float x1 = bbox[0], y1 = bbox[1], x2 = bbox[2], y2 = bbox[3];
      final float area = Math.max(0f, x2 - x1) * Math.max(0f, y2 - y1);
      final float areaRatio = area / Math.max(1f, (float) frameW * frameH);

      float avgProb = avg(probHist);
      float temporal = avg(frameCount);
      float avgTex = avg(texHist);
      float normTex = (texMax == texMin) ? avgTex : clamp01((avgTex - texMin) / (texMax - texMin));

      float left = x1, top = y1, right = frameW - x2, bottom = frameH - y2;
      float minBorder = Math.min(Math.min(left, top), Math.min(right, bottom));
      float border = Math.min(1f, minBorder / Math.max(1f, borderSafeDistance));

      float sizeFactor;
      if (areaRatio < minAreaRatio) {
         sizeFactor = areaRatio / Math.max(1e-6f, minAreaRatio);
      } else if (areaRatio > maxAreaRatio) {
         float denom = Math.max(1e-6f, 1f - maxAreaRatio);
         sizeFactor = Math.max(0f, (1f - areaRatio) / denom);
      } else {
         sizeFactor = 1f;
      }

      float total = wProb + wTemporal + wTexture + wBorder + wSize;
      score = (wProb * avgProb + wTemporal * temporal + wTexture * normTex + wBorder * border + wSize * sizeFactor)
              / Math.max(1e-6f, total);
      return score;
   }

   // ---- getters used by BoTSORTTracker / executor ----
   public int getTargetId()     { return targetId; }
   public float[] getBbox()     { return bbox; }
   public String getName()      { return name; }
   public int getMissedFrames() { return missedFrames; }
   public float getScore()      { return score; }

   // ---- helpers ----
   private static float avg(Iterable<? extends Number> xs) {
      float s = 0f; int n = 0; for (Number x : xs) { s += x.floatValue(); n++; }
      return n == 0 ? 0f : s / n;
   }
   private static float clamp01(float v) { return Math.max(0f, Math.min(1f, v)); }

   private <T> void pushBounded(ArrayDeque<T> q, T v) {
      q.addLast(v);
      while (q.size() > historySize) q.removeFirst(); // use per-track cap
   }
}
