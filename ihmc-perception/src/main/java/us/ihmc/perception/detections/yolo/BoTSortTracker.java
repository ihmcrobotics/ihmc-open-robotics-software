package us.ihmc.perception.detections.yolo;

import java.util.*;
import static java.lang.Math.*;

/**
 * Minimal BoT-SORT / ByteTrack-style tracker (NO ReID, NO GMC).
 *
 * Key behaviors matched to the common BoT-SORT/ByteTrack pipeline:
 *  - Split detections into HIGH and LOW confidence sets.
 *  - Predict all tracks with a KF.
 *  - First association: (Tracked + Lost)  <-> HIGH detections  (IoU cost + optional score fusion)
 *  - Second association: remaining TRACKED <-> LOW detections (IoU only, looser threshold)
 *  - Unconfirmed handling: tracks seen only once get matched separately; otherwise removed.
 *  - New track init from remaining HIGH detections (>= newTrackThresh)
 *  - Lost tracks are kept up to trackBuffer frames, then removed.
 *
 * IMPORTANT:
 *  - matchThresh in the python code is a COST threshold (for 1-IoU), not an IoU threshold.
 *    Here we follow that convention:
 *       cost = 1 - IoU
 *       accept match if cost <= matchThresh
 *
 * Dependencies/assumptions:
 *  - KalmanFilter is your xywh KF:
 *      state: [cx, cy, w, h, vcx, vcy, vw, vh]
 *      methods: initiate(xywh), predict(), update(xywh), getMean()
 */

public class BoTSortTracker
{
   // ---------------- Tunables (ByteTrack / BoT-SORT style) ----------------
   /** Detections >= trackHighThresh go to first association. */
   public float trackHighThresh = 0.6f;

   /** Detections in [trackLowThresh, trackHighThresh) go to second association. */
   public float trackLowThresh = 0.1f;

   /** Remaining unmatched HIGH detections >= newTrackThresh start new tracks. */
   public float newTrackThresh = 0.7f;

   /**
    * Association threshold in COST space (python uses lapjv cost_limit=match_thresh).
    * cost = 1 - IoU. So:
    * matchThresh=0.8  allows IoU >= 0.2
    * matchThresh=0.3  allows IoU >= 0.7
    */
   public float matchThresh = 0.8f;

   /** Second-stage association threshold for LOW detections (often ~0.5 in codebases). */
   public float secondMatchThresh = 0.5f;

   /** Unconfirmed association threshold (often ~0.7 in codebases). */
   public float unconfirmedMatchThresh = 0.7f;

   /** Frames to keep LOST tracks before removing. */
   public int trackBuffer = 30;

   private int baseTrackBuffer = 30; // unscaled "30fps-equivalent" buffer

   public void setTrackBuffer(int trackBufferAt30fps)
   {
      this.baseTrackBuffer = trackBufferAt30fps;
      this.trackBuffer = trackBufferAt30fps;
   }

   /** Call ONCE at init (e.g., when you know camera fps). */
   public void setFrameRate(float fps)
   {
      this.trackBuffer = Math.max(1, Math.round((fps / 30.0f) * baseTrackBuffer));
   }

   /** If true, fuse detection score into cost (similar to matching.fuse_score). */
   public boolean fuseScore = true;

   // ---------------- Internal state ----------------
   private int nextTrackId = 1;
   private int frameId = 0;

   private final List<STrack> tracked = new ArrayList<>();   // active confirmed tracks
   private final List<STrack> lost = new ArrayList<>();      // temporarily lost
   private final List<STrack> removed = new ArrayList<>();   // permanently removed

   public int getTrackedCount()
   {
      return tracked.size();
   }

   public int getLostCount()
   {
      return lost.size();
   }

   public int getRemovedCount()
   {
      return removed.size();
   }

   /**
    * Update tracker with detections for the current frame.
    * This sets track IDs into detections via d.setTrackId(...).
    */
   public void update(List<? extends TrackableDetection> detections)
   {
      frameId++;

      // ---- Split detections: high and low ----
      List<TrackableDetection> high = new ArrayList<>();
      List<TrackableDetection> low = new ArrayList<>();
      for (TrackableDetection d : detections)
      {
         double c = d.getConfidence();
         if (c >= trackHighThresh)
            high.add(d);
         else if (c >= trackLowThresh)
            low.add(d);
      }

      // ---- Split tracks into confirmed tracked vs unconfirmed (only 1 hit) ----
      List<STrack> unconfirmed = new ArrayList<>();
      List<STrack> confirmedTracked = new ArrayList<>();
      for (STrack t : tracked)
      {
         if (!t.isActivated)
            unconfirmed.add(t);
         else
            confirmedTracked.add(t);
      }

      // ---- Predict all states ----
      for (STrack t : confirmedTracked)
         t.predict();
      for (STrack t : unconfirmed)
         t.predict();
      for (STrack t : lost)
         t.predict();

      // ---- First association: (confirmed tracked + lost) <-> HIGH ----
      List<STrack> strackPool = new ArrayList<>(confirmedTracked);
      strackPool.addAll(lost);

      AssocResult a1 = associate(strackPool, high, matchThresh, fuseScore);

      List<STrack> activated = new ArrayList<>();
      List<STrack> refound = new ArrayList<>();
      List<STrack> newlyLost = new ArrayList<>();
      List<STrack> newlyRemoved = new ArrayList<>();

      // Apply matches
      for (IntPair m : a1.matches)
      {
         STrack t = strackPool.get(m.i);
         TrackableDetection d = high.get(m.j);

         if (t.state == TrackState.Tracked)
         {
            t.update(d, frameId);
            activated.add(t);
         }
         else
         {
            // was Lost -> refind
            t.reActivate(d, frameId, false);
            refound.add(t);
         }
         d.setTrackId(t.id);
      }

      // Unmatched tracks from pool after high association
      List<STrack> uTrack1 = new ArrayList<>();
      for (int ti : a1.unmatchedTrackIdx)
         uTrack1.add(strackPool.get(ti));

      // ---- Second association: remaining TRACKED (only) <-> LOW ----
      // In python they restrict to those with state==Tracked
      List<STrack> rTracked = new ArrayList<>();
      for (STrack t : uTrack1)
         if (t.state == TrackState.Tracked)
            rTracked.add(t);

      AssocResult a2 = associate(rTracked, low, secondMatchThresh, false);

      for (IntPair m : a2.matches)
      {
         STrack t = rTracked.get(m.i);
         TrackableDetection d = low.get(m.j);
         t.update(d, frameId);
         activated.add(t);
         d.setTrackId(t.id);
      }

      // Remaining unmatched rTracked -> mark lost
      Set<Integer> matchedRTracked = new HashSet<>();
      for (IntPair m : a2.matches)
         matchedRTracked.add(m.i);
      for (int i = 0; i < rTracked.size(); i++)
      {
         if (!matchedRTracked.contains(i))
         {
            STrack t = rTracked.get(i);
            t.markLost();
            newlyLost.add(t);
         }
      }

      // ---- Unconfirmed association with remaining HIGH detections ----
      // Use the leftover HIGH detections from a1
      List<TrackableDetection> remainingHigh = new ArrayList<>();
      for (int di : a1.unmatchedDetIdx)
         remainingHigh.add(high.get(di));

      AssocResult a3 = associate(unconfirmed, remainingHigh, unconfirmedMatchThresh, fuseScore);

      for (IntPair m : a3.matches)
      {
         STrack t = unconfirmed.get(m.i);
         TrackableDetection d = remainingHigh.get(m.j);
         t.update(d, frameId);
         activated.add(t);
         d.setTrackId(t.id);
      }

      // Unmatched unconfirmed -> remove
      Set<Integer> matchedUnconfirmed = new HashSet<>();
      for (IntPair m : a3.matches)
         matchedUnconfirmed.add(m.i);
      for (int i = 0; i < unconfirmed.size(); i++)
      {
         if (!matchedUnconfirmed.contains(i))
         {
            STrack t = unconfirmed.get(i);
            t.markRemoved();
            newlyRemoved.add(t);
         }
      }

      // ---- Init new tracks from remaining HIGH detections (unmatched in a3) ----
      Set<Integer> matchedRemainingHigh = new HashSet<>();
      for (IntPair m : a3.matches)
         matchedRemainingHigh.add(m.j);

      for (int j = 0; j < remainingHigh.size(); j++)
      {
         if (matchedRemainingHigh.contains(j))
            continue;

         TrackableDetection d = remainingHigh.get(j);
         if (d.getConfidence() < newTrackThresh)
            continue;

         STrack nt = new STrack(nextTrackId++, d, frameId);
         activated.add(nt);
         d.setTrackId(nt.id);
      }

      // ---- Age out old lost tracks ----
      for (STrack t : lost)
      {
         if (frameId - t.endFrame() > trackBuffer)
         {
            t.markRemoved();
            newlyRemoved.add(t);
         }
      }

      // ---- Merge / update global lists (same spirit as python joint/sub/remove-dup) ----
      // tracked = (tracked that are Tracked) U activated U refound
      // lost    = (lost - tracked) U newlyLost  (then remove removed)
      // removed = removed U newlyRemoved

      // Keep only currently Tracked in tracked
      tracked.removeIf(t -> t.state != TrackState.Tracked);

      // Add activated + refound
      jointUniqueById(tracked, activated);
      jointUniqueById(tracked, refound);

      // Update lost: remove anything that is now tracked
      subById(lost, tracked);

      // Add new lost
      jointUniqueById(lost, newlyLost);

      // Remove anything that is removed
      subById(lost, newlyRemoved);

      // Add removed
      jointUniqueById(removed, newlyRemoved);

      // Optional: remove duplicates between tracked and lost
      removeDuplicates(tracked, lost, 0.15f);

      // Final invariants
      tracked.removeIf(t -> t.state != TrackState.Tracked);
      lost.removeIf(t -> t.state != TrackState.Lost);
      removed.removeIf(t -> t.state != TrackState.Removed);

      // ensure no id appears in both lists
      subById(lost, tracked);
   }

   // ---------------- Association ----------------

   private AssocResult associate(List<STrack> tracks, List<? extends TrackableDetection> dets, float costThresh, boolean fuseScoreHere)
   {
      AssocResult res = new AssocResult();

      if (tracks.isEmpty())
      {
         for (int j = 0; j < dets.size(); j++)
            res.unmatchedDetIdx.add(j);
         return res;
      }
      if (dets.isEmpty())
      {
         for (int i = 0; i < tracks.size(); i++)
            res.unmatchedTrackIdx.add(i);
         return res;
      }

      // cost = 1 - IoU; optionally fuse score like python matching.fuse_score:
      // fuse_cost = 1 - (IoU * score)
      float[][] cost = new float[tracks.size()][dets.size()];
      for (int i = 0; i < tracks.size(); i++)
      {
         float[] tb = tracks.get(i).getTlbr(); // predicted
         for (int j = 0; j < dets.size(); j++)
         {
            TrackableDetection d = dets.get(j);
            float iou = iou(tb[0], tb[1], tb[2], tb[3], d.getX1(), d.getY1(), d.getX2(), d.getY2());
            float c;
            if (fuseScoreHere)
            {
               float s = (float) clamp(d.getConfidence(), 0.0, 1.0);
               c = 1.0f - (iou * s);
            }
            else
            {
               c = 1.0f - iou;
            }
            cost[i][j] = c;
         }
      }

   // LAPJV (Hungarian variant) with cost_limit=thresh.
   int[] assignment = hungarianWithCostLimit(cost, costThresh);

   boolean[] detUsed = new boolean[dets.size()];
   for(
   int i = 0; i<tracks.size();i++)

   {
      int j = assignment[i];
      if (j >= 0 && j < dets.size())
      {
         // accepted match (guaranteed cost <= costThresh)
         detUsed[j] = true;
         res.matches.add(new IntPair(i, j));
      }
      else
      {
         res.unmatchedTrackIdx.add(i);
      }
   }
   for(
   int j = 0; j<dets.size();j++)

   {
      if (!detUsed[j])
         res.unmatchedDetIdx.add(j);
   }

   return res;
   }
   /**
    * Hungarian assignment with a BoT-SORT-like cost limit.
    *
    * Input: cost[n][m] where n = tracks, m = detections
    * Output: assignment array of length n: assignment[i] = matched det index, or -1 if unmatched.
    *
    * We implement "cost_limit" by adding n dummy columns with cost = (costLimit + eps),
    * so each track can always be assigned to something. If Hungarian chooses dummy
    * (or a real det whose cost > costLimit), we treat it as unmatched.
    */
   private static int[] hungarianWithCostLimit(float[][] cost, float costLimit)
   {
      final int n = cost.length;
      final int m = (n == 0) ? 0 : cost[0].length;
      int[] result = new int[n];
      Arrays.fill(result, -1);

      if (n == 0) return result;
      if (m == 0) return result;

      // Augment with dummy columns so every track can be "assigned" (meaning: unmatched)
      // This mimics LAPJV cost_limit behavior.
      final float eps = 1e-3f;
      final float dummyCost = costLimit + eps;

      int M = m + n; // real dets + one dummy per track
      double[][] a = new double[n][M];

      final double INF = 1e12;

      for (int i = 0; i < n; i++)
      {
         // real detection columns
         for (int j = 0; j < m; j++)
         {
            float c = cost[i][j];
            // Disallow edges above costLimit by setting them to INF
            a[i][j] = (c <= costLimit) ? c : INF;
         }
         // dummy columns (always allowed)
         for (int j = m; j < M; j++)
         {
            a[i][j] = dummyCost;
         }
      }

      // Solve assignment: rows->cols (min cost)
      int[] colForRow = hungarianMinCost(a); // length n, each in [0, M)

      // Interpret
      for (int i = 0; i < n; i++)
      {
         int j = colForRow[i];
         if (j >= 0 && j < m)
         {
            // Because we INF-masked cost>limit, any real assignment is <=limit
            result[i] = j;
         }
         else
         {
            result[i] = -1; // dummy => unmatched
         }
      }

      return result;
   }

   /**
    * Hungarian algorithm for rectangular matrix (n rows, M cols), n <= M works best.
    * Returns colForRow[i] in [0, M).
    *
    * Implementation is the classical potentials method (O(n^2 * M)) with 1-indexing.
    */
   private static int[] hungarianMinCost(double[][] a)
   {
      int n = a.length;
      int m = a[0].length;

      // If n > m, you should transpose; in our use n <= m (because of dummy cols)
      if (n > m)
         throw new IllegalArgumentException("Hungarian expects n <= m (rows <= cols).");

      double[] u = new double[n + 1];
      double[] v = new double[m + 1];
      int[] p = new int[m + 1];     // matching for columns: p[j] = row assigned to column j
      int[] way = new int[m + 1];

      for (int i = 1; i <= n; i++)
      {
         p[0] = i;
         int j0 = 0;

         double[] minv = new double[m + 1];
         boolean[] used = new boolean[m + 1];
         Arrays.fill(minv, Double.POSITIVE_INFINITY);
         Arrays.fill(used, false);

         do
         {
            used[j0] = true;
            int i0 = p[j0];
            int j1 = 0;
            double delta = Double.POSITIVE_INFINITY;

            for (int j = 1; j <= m; j++)
            {
               if (used[j]) continue;

               double cur = a[i0 - 1][j - 1] - u[i0] - v[j];
               if (cur < minv[j])
               {
                  minv[j] = cur;
                  way[j] = j0;
               }
               if (minv[j] < delta)
               {
                  delta = minv[j];
                  j1 = j;
               }
            }

            for (int j = 0; j <= m; j++)
            {
               if (used[j])
               {
                  u[p[j]] += delta;
                  v[j] -= delta;
               }
               else
               {
                  minv[j] -= delta;
               }
            }

            j0 = j1;

         } while (p[j0] != 0);

         // Augmenting
         do
         {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
         } while (j0 != 0);
      }

      // Build row->col assignment from p (col->row)
      int[] colForRow = new int[n];
      Arrays.fill(colForRow, -1);
      for (int j = 1; j <= m; j++)
      {
         int i = p[j];
         if (i >= 1 && i <= n)
            colForRow[i - 1] = j - 1;
      }
      return colForRow;
   }

   public static class TrackViz
   {
      public final int id;
      public final float x1, y1, x2, y2;

      public TrackViz(int id, float[] tlbr)
      {
         this.id = id;
         this.x1 = tlbr[0];
         this.y1 = tlbr[1];
         this.x2 = tlbr[2];
         this.y2 = tlbr[3];
      }
   }

   public List<TrackViz> getActiveTracks()
   {
      List<TrackViz> out = new ArrayList<>();
      for (STrack t : tracked)
      {
         if (t.state == TrackState.Tracked)
            out.add(new TrackViz(t.id, t.getTlbr()));
      }
      return out;
   }

   private static void jointUniqueById(List<STrack> dst, List<STrack> add)
   {
      // joint_stracks: A wins; only add B items whose id not in A.
      Map<Integer, STrack> map = new LinkedHashMap<>();
      for (STrack t : dst)
         map.put(t.id, t);

      for (STrack t : add)
         map.putIfAbsent(t.id, t);  // keep existing (A wins)

      dst.clear();
      dst.addAll(map.values());
   }

   private static void subById(List<STrack> a, List<STrack> b)
   {
      Set<Integer> bIds = new HashSet<>();
      for (STrack t : b) bIds.add(t.id);
      a.removeIf(t -> bIds.contains(t.id));
   }

   private static void removeDuplicates(List<STrack> tracked, List<STrack> lost, float iouDupThreshCost)
   {
      // python: pairs where cost < 0.15 => duplicate. Keep the one with longer age.
      // Here: compute pairwise cost and remove duplicates from the shorter-lived set.
      List<Integer> dupTracked = new ArrayList<>();
      List<Integer> dupLost = new ArrayList<>();

      for (int i = 0; i < tracked.size(); i++)
      {
         for (int j = 0; j < lost.size(); j++)
         {
            float[] a = tracked.get(i).getTlbr();
            float[] b = lost.get(j).getTlbr();
            float cost = 1.0f - iou(a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]);
            if (cost < iouDupThreshCost)
            {
               int timeA = tracked.get(i).frameId - tracked.get(i).startFrame;
               int timeB = lost.get(j).frameId - lost.get(j).startFrame;
               if (timeA > timeB) dupLost.add(j);
               else dupTracked.add(i);
            }
         }
      }

      // remove in reverse order
      dupTracked.sort(Comparator.reverseOrder());
      dupLost.sort(Comparator.reverseOrder());
      for (int idx : new LinkedHashSet<>(dupTracked)) if (idx >= 0 && idx < tracked.size()) tracked.remove(idx);
      for (int idx : new LinkedHashSet<>(dupLost))    if (idx >= 0 && idx < lost.size())    lost.remove(idx);
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

   private static double clamp(double v, double lo, double hi)
   {
      return max(lo, min(hi, v));
   }

   // ---------------- Data structures ----------------

   private static class AssocResult
   {
      List<IntPair> matches = new ArrayList<>();
      List<Integer> unmatchedTrackIdx = new ArrayList<>();
      List<Integer> unmatchedDetIdx = new ArrayList<>();
   }

   private static class IntPair
   {
      final int i; // track index
      final int j; // det index
      IntPair(int i, int j) { this.i = i; this.j = j; }
   }

   private static class Triplet
   {
      final int i, j;
      final float cost;
      Triplet(int i, int j, float cost) { this.i = i; this.j = j; this.cost = cost; }
   }

   // ---------------- Track state ----------------

   private enum TrackState
   {
      New,
      Tracked,
      Lost,
      Removed
   }

   // ---------------- Track (KF + lifecycle) ----------------

   private static class STrack
   {
      final int id;
      final KalmanFilter kf = new KalmanFilter(); // your xywh KF

      TrackState state = TrackState.New;

      // lifecycle bookkeeping
      boolean isActivated = false;
      int startFrame = 0;
      int frameId = 0;
      int timeSinceUpdate = 0;

      // cached predicted box in tlbr
      private float[] tlbr = new float[4];

      STrack(int id, TrackableDetection det, int frameId)
      {
         this.id = id;
         activate(det, frameId);
      }

      void activate(TrackableDetection det, int fid)
      {
         float[] xywh = detToXYWH(det);
         kf.initiate(xywh);

         this.startFrame = fid;
         this.frameId = fid;
         this.timeSinceUpdate = 0;

         this.state = TrackState.Tracked;

         // In BoT-SORT/ByteTrack code, tracks can be "activated" after first successful update.
         // Many codebases set isActivated true immediately (or only when frame==1).
         this.isActivated = false;

         this.tlbr = meanToTLBR(kf.getMean());
      }

      void reActivate(TrackableDetection det, int fid, boolean newId)
      {
         // We won't actually "newId" here unless you want that behavior.
         update(det, fid);
         this.state = TrackState.Tracked;
         this.isActivated = true;
      }

      void predict()
      {
         if (state == TrackState.Removed)
            return;

         kf.predict();
         tlbr = meanToTLBR(kf.getMean());
         timeSinceUpdate++;
      }

      void update(TrackableDetection det, int fid)
      {
         float[] xywh = detToXYWH(det);
         kf.update(xywh);
         tlbr = meanToTLBR(kf.getMean());

         frameId = fid;
         timeSinceUpdate = 0;

         state = TrackState.Tracked;
         isActivated = true;
      }

      void markLost()
      {
         state = TrackState.Lost;
      }

      void markRemoved()
      {
         state = TrackState.Removed;
      }

      int endFrame()
      {
         return frameId;
      }

      float[] getTlbr()
      {
         return tlbr;
      }

      // ---- conversions ----

      static float[] detToXYWH(TrackableDetection d)
      {
         float x1 = d.getX1();
         float y1 = d.getY1();
         float x2 = d.getX2();
         float y2 = d.getY2();

         float w = max(1f, x2 - x1);
         float h = max(1f, y2 - y1);
         float cx = x1 + 0.5f * w;
         float cy = y1 + 0.5f * h;

         // THIS MUST BE (cx, cy, w, h) for BoT-SORT's KF variant you showed.
         return new float[] {cx, cy, w, h};
      }

      static float[] meanToTLBR(float[] mean)
      {
         // mean[0]=cx mean[1]=cy mean[2]=w mean[3]=h (as in your python KF)
         float cx = mean[0], cy = mean[1], w = max(1f, mean[2]), h = max(1f, mean[3]);

         float x1 = cx - 0.5f * w;
         float y1 = cy - 0.5f * h;
         float x2 = cx + 0.5f * w;
         float y2 = cy + 0.5f * h;

         return new float[] {x1, y1, x2, y2};
      }
   }
}