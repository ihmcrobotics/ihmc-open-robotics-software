package us.ihmc.perception.detections.yolo;

import java.util.*;
import static java.lang.Math.*;

public class BoTSortTracker
{
   // ---- Tunables (mirror BoT-SORT/ByteTrack knobs) ----
   public float trackHighThresh = 0.6f;
   public float trackLowThresh  = 0.1f;
   public float newTrackThresh  = 0.7f;
   public float matchThresh     = 0.8f; // threshold on IoU (or 1-cost)
   public int trackBuffer       = 30;   // frames to keep "lost" tracks

   // If true, fuse detection score into matching cost (BoT-SORT has this option)
   public boolean fuseScore = false;

   private int nextTrackId = 1;

   // Active + lost tracks
   private final List<STrack> tracked = new ArrayList<>();
   private final List<STrack> lost = new ArrayList<>();

   public void update(List<? extends TrackableDetection> detections)
   {
      // Split detections into high/low confidence like ByteTrack
      List<TrackableDetection> high = new ArrayList<>();
      List<TrackableDetection> low  = new ArrayList<>();
      for (TrackableDetection d : detections)
      {
         if (d.getConfidence() >= trackHighThresh) high.add(d);
         else if (d.getConfidence() >= trackLowThresh) low.add(d);
      }

      // 1) Predict tracks (Kalman)
      for (STrack t : tracked) t.predict();
      for (STrack t : lost)    t.predict();

      // Merge tracked + lost into "candidates" for matching
      List<STrack> candidates = new ArrayList<>(tracked);
      candidates.addAll(lost);

      // 2) First association: candidates ↔ high conf
      MatchResult m1 = associate(candidates, high);

      // Update matched tracks
      Set<STrack> activated = new HashSet<>();
      for (Pair p : m1.matches)
      {
         STrack t = candidates.get(p.trackIndex);
         TrackableDetection d = high.get(p.detIndex);
         t.update(d);
         activated.add(t);
         d.setTrackId(t.id);
      }

      // Unmatched tracks after first association
      List<STrack> uTracks1 = new ArrayList<>();
      for (int i : m1.unmatchedTrackIdx) uTracks1.add(candidates.get(i));

      // 3) Second association: remaining tracks ↔ low conf (ByteTrack trick)
      MatchResult m2 = associate(uTracks1, low);

      Set<STrack> activated2 = new HashSet<>();
      for (Pair p : m2.matches)
      {
         STrack t = uTracks1.get(p.trackIndex);
         TrackableDetection d = low.get(p.detIndex);
         t.update(d);
         activated2.add(t);
         d.setTrackId(t.id);
      }

      // Tracks that are still unmatched -> mark lost
      Set<STrack> stillUnmatched = new HashSet<>();
      for (int i : m2.unmatchedTrackIdx) stillUnmatched.add(uTracks1.get(i));

      // 4) Create new tracks from unmatched HIGH detections (≥ newTrackThresh)
      for (int detIdx : m1.unmatchedDetIdx)
      {
         TrackableDetection d = high.get(detIdx);
         if (d.getConfidence() >= newTrackThresh)
         {
            STrack nt = new STrack(nextTrackId++, d);
            tracked.add(nt);
            d.setTrackId(nt.id);
         }
      }

      // 5) Update track lists: move lost/active, age out old lost
      // Rebuild tracked/lost cleanly
      // A track is "active" if it was updated this frame, else it becomes lost.
      List<STrack> newTracked = new ArrayList<>();
      List<STrack> newLost = new ArrayList<>();

      // Candidates includes both tracked+lost; we want to keep identity stable
      // Add all tracks that were activated in either stage to active
      for (STrack t : candidates)
      {
         if (t.timeSinceUpdate == 0) newTracked.add(t);
         else newLost.add(t);
      }

      // Apply buffer: remove very old lost tracks
      newLost.removeIf(t -> t.timeSinceUpdate > trackBuffer);

      tracked.clear();
      lost.clear();
      tracked.addAll(newTracked);
      lost.addAll(newLost);
   }

   // ---------------- Matching ----------------

   private MatchResult associate(List<STrack> tracks, List<? extends TrackableDetection> dets)
   {
      MatchResult res = new MatchResult();
      if (tracks.isEmpty())
      {
         res.unmatchedDetIdx.addAll(range(0, dets.size()));
         return res;
      }
      if (dets.isEmpty())
      {
         res.unmatchedTrackIdx.addAll(range(0, tracks.size()));
         return res;
      }

      // Cost = 1 - IoU (lower is better)
      float[][] cost = new float[tracks.size()][dets.size()];
      for (int i = 0; i < tracks.size(); i++)
      {
         float[] tb = tracks.get(i).getTlbr(); // predicted box
         for (int j = 0; j < dets.size(); j++)
         {
            TrackableDetection d = dets.get(j);
            float iou = iou(tb[0], tb[1], tb[2], tb[3], d.getX1(), d.getY1(), d.getX2(), d.getY2());
            float c = 1.0f - iou;
            if (fuseScore)
               c *= (2.0f - d.getConfidence()); // simple fusion (lower conf => higher cost)
            cost[i][j] = c;
         }
      }

      // Greedy matching by minimal cost (good baseline; replace with Hungarian later if needed)
      boolean[] trackUsed = new boolean[tracks.size()];
      boolean[] detUsed = new boolean[dets.size()];

      // Flatten all pairs
      List<Triplet> pairs = new ArrayList<>();
      for (int i = 0; i < tracks.size(); i++)
         for (int j = 0; j < dets.size(); j++)
            pairs.add(new Triplet(i, j, cost[i][j]));

      pairs.sort(Comparator.comparingDouble(a -> a.cost));

      for (Triplet t : pairs)
      {
         if (trackUsed[t.i] || detUsed[t.j]) continue;

         float iouVal = 1.0f - t.cost;
         if (iouVal < matchThresh) continue;

         trackUsed[t.i] = true;
         detUsed[t.j] = true;
         res.matches.add(new Pair(t.i, t.j));
      }

      for (int i = 0; i < tracks.size(); i++) if (!trackUsed[i]) res.unmatchedTrackIdx.add(i);
      for (int j = 0; j < dets.size(); j++) if (!detUsed[j]) res.unmatchedDetIdx.add(j);

      return res;
   }

   private static List<Integer> range(int start, int end)
   {
      List<Integer> r = new ArrayList<>();
      for (int i = start; i < end; i++) r.add(i);
      return r;
   }

   private static float iou(float ax1, float ay1, float ax2, float ay2,
                            float bx1, float by1, float bx2, float by2)
   {
      float ix1 = max(ax1, bx1);
      float iy1 = max(ay1, by1);
      float ix2 = min(ax2, bx2);
      float iy2 = min(ay2, by2);

      float iw = max(0f, ix2 - ix1);
      float ih = max(0f, iy2 - iy1);
      float inter = iw * ih;

      float a = max(0f, ax2 - ax1) * max(0f, ay2 - ay1);
      float b = max(0f, bx2 - bx1) * max(0f, by2 - by1);
      float union = a + b - inter;
      return union <= 0f ? 0f : inter / union;
   }

   // ---------------- Data structures ----------------
   private static class MatchResult
   {
      List<Pair> matches = new ArrayList<>();
      List<Integer> unmatchedTrackIdx = new ArrayList<>();
      List<Integer> unmatchedDetIdx = new ArrayList<>();
   }

   private static class Pair
   {
      int trackIndex;
      int detIndex;
      Pair(int ti, int di) { trackIndex = ti; detIndex = di; }
   }

   private static class Triplet
   {
      int i, j;
      float cost;
      Triplet(int i, int j, float c) { this.i=i; this.j=j; this.cost=c; }
   }

   // ---------------- Track (Kalman + lifecycle) ----------------
   private static class STrack
   {
      final int id;
      final KalmanFilterXYAH kf = new KalmanFilterXYAH();

      // predicted bbox tlbr cached
      private float[] tlbr = new float[4];

      int timeSinceUpdate = 0;

      STrack(int id, TrackableDetection det)
      {
         this.id = id;
         float[] xyah = toXYAH(det.getX1(), det.getY1(), det.getX2(), det.getY2());
         kf.initiate(xyah);
         tlbr = fromXYAH(kf.getMean());
      }

      void predict()
      {
         kf.predict();
         tlbr = fromXYAH(kf.getMean());
         timeSinceUpdate++;
      }

      void update(TrackableDetection det)
      {
         float[] xyah = toXYAH(det.getX1(), det.getY1(), det.getX2(), det.getY2());
         kf.update(xyah);
         tlbr = fromXYAH(kf.getMean());
         timeSinceUpdate = 0;
      }

      float[] getTlbr()
      {
         return tlbr;
      }

      // (x1,y1,x2,y2) -> (cx,cy,a,h) where a=w/h
      static float[] toXYAH(float x1, float y1, float x2, float y2)
      {
         float w = max(1f, x2 - x1);
         float h = max(1f, y2 - y1);
         float cx = x1 + w / 2f;
         float cy = y1 + h / 2f;
         float a = w / h;
         return new float[] {cx, cy, a, h};
      }

      // mean xyah -> tlbr
      static float[] fromXYAH(float[] mean)
      {
         float cx = mean[0], cy = mean[1], a = mean[2], h = mean[3];
         float w = a * h;
         float x1 = cx - w / 2f;
         float y1 = cy - h / 2f;
         float x2 = cx + w / 2f;
         float y2 = cy + h / 2f;
         return new float[] {x1, y1, x2, y2};
      }
   }
}