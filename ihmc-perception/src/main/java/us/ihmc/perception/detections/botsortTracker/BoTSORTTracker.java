package us.ihmc.perception.detections.botsortTracker;

import java.util.*;
import static java.lang.Math.max;
import static java.lang.Math.min;

public final class BoTSORTTracker {
   private final int historySize;
   private final float startThresh, stopThresh;
   private final int minFrameCount, maxMissedFrames;
   private final float minAreaRatio, maxAreaRatio;
   private final float texMin, texMax;
   private final float borderSafeDistance;
   private final float wProb, wTemporal, wTexture, wBorder, wSize;

   private final Map<Integer, BoTSORTTrack> targets = new HashMap<>();
   private int nextId = 0;

   public BoTSORTTracker(
         int historySize,
         float startThresh, float stopThresh,
         int minFrameCount, int maxMissedFrames,
         float minAreaRatio, float maxAreaRatio,
         float texMin, float texMax,
         float borderSafeDistance,
         float wProb, float wTemporal, float wTexture, float wBorder, float wSize) {
      this.historySize = historySize;
      this.startThresh = startThresh;
      this.stopThresh = stopThresh;
      this.minFrameCount = minFrameCount;
      this.maxMissedFrames = maxMissedFrames;
      this.minAreaRatio = minAreaRatio;
      this.maxAreaRatio = maxAreaRatio;
      this.texMin = texMin; this.texMax = texMax;
      this.borderSafeDistance = borderSafeDistance;
      this.wProb = wProb; this.wTemporal = wTemporal; this.wTexture = wTexture; this.wBorder = wBorder; this.wSize = wSize;
   }

   public static double iou(float[] a, float[] b) {
      float x1 = max(a[0], b[0]);
      float y1 = max(a[1], b[1]);
      float x2 = min(a[2], b[2]);
      float y2 = min(a[3], b[3]);
      float inter = max(0f, x2 - x1) * max(0f, y2 - y1);
      float area1 = max(0f, a[2] - a[0]) * max(0f, a[3] - a[1]);
      float area2 = max(0f, b[2] - b[0]) * max(0f, b[3] - b[1]);
      float uni = area1 + area2 - inter;
      return uni <= 0f ? 0.0 : (inter / uni);
   }

   /** Update tracker with current-frame detections (2D only). */
   public List<BoTSORTTrack> update(
         int frameW, int frameH,
         List<float[]> bboxes, List<Float> probs, List<String> names) {

      Set<Integer> updated = new HashSet<>();
      Set<Integer> usedTargets = new HashSet<>();

      // Assign detections → existing targets (class-gated IoU)
      for (int i = 0; i < bboxes.size(); i++) {
         float[] bb = bboxes.get(i);
         float prob = probs.get(i);
         String name = names.get(i);

         BoTSORTTrack best = null; double bestIoU = 0.0;
         for (BoTSORTTrack t : targets.values()) {
            if (usedTargets.contains(t.getTargetId())) continue;
            if (!Objects.equals(t.getName(), name)) continue;
            double v = iou(bb, t.getBbox());
            if (v > 0.5 && v > bestIoU) { bestIoU = v; best = t; }
         }

         if (best != null) {
            best.update(bb, -1, prob, name, 0f);
            updated.add(best.getTargetId());
            usedTargets.add(best.getTargetId());
         } else {
            BoTSORTTrack t = new BoTSORTTrack(nextId, bb, -1, prob, name, 0f, historySize);
            targets.put(nextId, t);
            updated.add(nextId);
            usedTargets.add(nextId);
            nextId++;
         }
      }

      // Age & prune
      Iterator<Map.Entry<Integer, BoTSORTTrack>> it = targets.entrySet().iterator();
      Map<Integer, Float> scores = new HashMap<>();
      while (it.hasNext()) {
         BoTSORTTrack t = it.next().getValue();
         if (!updated.contains(t.getTargetId())) {
            t.markMissed();
            if (t.getMissedFrames() > maxMissedFrames) { it.remove(); continue; }
         }
         float s = t.computeScore(frameW, frameH,
                                  minAreaRatio, maxAreaRatio, texMin, texMax, borderSafeDistance,
                                  wProb, wTemporal, wTexture, wBorder, wSize);
         scores.put(t.getTargetId(), s);
         if (s < stopThresh) { it.remove(); }
      }

      // Decide which to publish
      List<BoTSORTTrack> published = new ArrayList<>();
      for (BoTSORTTrack t : targets.values()) {
         float s = scores.getOrDefault(t.getTargetId(), t.getScore());
         if (s >= startThresh && t.framesSeen() >= minFrameCount && t.getMissedFrames() <= maxMissedFrames) {
            published.add(t);
         }
      }
      return published;
   }
}
