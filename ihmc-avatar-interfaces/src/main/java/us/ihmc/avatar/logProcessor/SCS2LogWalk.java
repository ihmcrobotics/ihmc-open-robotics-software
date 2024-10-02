package us.ihmc.avatar.logProcessor;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;

public class SCS2LogWalk
{
   private long walkStartTick = -1;
   private final Point2D walkStart = new Point2D();
   private final ArrayList<SCS2LogDataFootstep> footsteps = new ArrayList<>();
   private final RecyclingArrayList<Point2D> coms = new RecyclingArrayList<>(Point2D::new);

   private boolean endedWithFall = false;
   private int initialWorkingCounterMismatch = -1;
   private int workingCounterMismatch = 0;

   public void update(double currentTime, long tick, YoInteger yoWorkingCounterMismatch)
   {
      if (initialWorkingCounterMismatch < 0)
         initialWorkingCounterMismatch = yoWorkingCounterMismatch.getIntegerValue();

      workingCounterMismatch = yoWorkingCounterMismatch.getIntegerValue() - initialWorkingCounterMismatch;
   }

   public void setWalkStartTick(long walkStartTick)
   {
      this.walkStartTick = walkStartTick;
   }

   public long getWalkStartTick()
   {
      return walkStartTick;
   }

   public Point2D getWalkStart()
   {
      return walkStart;
   }

   public ArrayList<SCS2LogDataFootstep> getFootsteps()
   {
      return footsteps;
   }

   public boolean isEndedWithFall()
   {
      return endedWithFall;
   }

   public void setEndedWithFall(boolean endedWithFall)
   {
      this.endedWithFall = endedWithFall;
   }

   public RecyclingArrayList<Point2D> getComs()
   {
      return coms;
   }

   public int getInitialWorkingCounterMismatch()
   {
      return initialWorkingCounterMismatch;
   }
}
