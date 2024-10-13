package us.ihmc.avatar.logProcessor;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;

public class SCS2LogWalk
{
   private long walkStartTick = -1;
   private final Point2D walkStart = new Point2D();
   private final ArrayList<SCS2LogFootstep> footsteps = new ArrayList<>();
   private final TDoubleArrayList times = new TDoubleArrayList();
   private final RecyclingArrayList<Pose3D> pelvisPoses = new RecyclingArrayList<>(Pose3D::new);
   private final SideDependentList<RecyclingArrayList<Pose3D>> handPoses = new SideDependentList<>(new RecyclingArrayList<>(Pose3D::new),
                                                                                                   new RecyclingArrayList<>(Pose3D::new));
   private final RecyclingArrayList<Point2D> coms = new RecyclingArrayList<>(Point2D::new);
   private final RecyclingArrayList<Point2D> icps = new RecyclingArrayList<>(Point2D::new);

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

   public ArrayList<SCS2LogFootstep> getFootsteps()
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

   public TDoubleArrayList getTimes()
   {
      return times;
   }

   public RecyclingArrayList<Pose3D> getPelvisPoses()
   {
      return pelvisPoses;
   }

   public SideDependentList<RecyclingArrayList<Pose3D>> getHandPoses()
   {
      return handPoses;
   }

   public RecyclingArrayList<Point2D> getComs()
   {
      return coms;
   }

   public RecyclingArrayList<Point2D> getIcps()
   {
      return icps;
   }

   public int getInitialWorkingCounterMismatch()
   {
      return initialWorkingCounterMismatch;
   }
}
