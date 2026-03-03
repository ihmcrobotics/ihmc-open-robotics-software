package us.ihmc.perception.detections.yolo;

import us.ihmc.perception.RawImage;

import java.util.ArrayDeque;
import java.util.Deque;

public class Target2D
{
   public final int targetId;       // stable id
   public int lastTrackId;          // source tracker id (may change)
   public String name;

   // Latest observation (retained)
   public RawImage latestMask;      // may be null
   public float[] latestBbox;       // [x1,y1,x2,y2] (defensive-copied)

   // Lifecycle / scoring history
   public int missedFrames = 0;
   public final Deque<Integer> frameCount;   // 1 seen, 0 missed
   public final Deque<Float> probHistory;
   public final Deque<Float> textureHistory;

   public float score = 0.0f;

   public Target2D(int targetId,
                   int initialTrackId,
                   String name,
                   RawImage mask,          // may be null
                   float[] bbox,           // expected length 4
                   float prob,
                   float texture,
                   int historySize)
   {
      this.targetId = targetId;
      this.lastTrackId = initialTrackId;
      this.name = name;

      // retain mask
      this.latestMask = (mask == null) ? null : mask.get();

      // defensive copy bbox (prevents caller-side mutation bugs)
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
                      RawImage mask,        // may be null
                      float[] bbox,         // expected length 4
                      float prob,
                      float texture,
                      int historySize)
   {
      this.lastTrackId = trackId;
      this.name = name;

      // defensive copy bbox
      this.latestBbox = (bbox == null) ? null : bbox.clone();

      // swap retained mask safely
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