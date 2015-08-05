package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePose;

public class UpcomingFootstepList
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FootstepProvider footstepProvider;

   private final IntegerYoVariable footstepIndex = new IntegerYoVariable("footstepIndex", registry);
   private final IntegerYoVariable numberOfLocalFootsteps = new IntegerYoVariable("numberOfLocalFootsteps", registry);

   private final List<Footstep> outputFootstepList = new ArrayList<Footstep>();
   private final List<Footstep> localFootstepList = new ArrayList<Footstep>();

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFramePose nextFootstepPose = new YoFramePose("nextFootstep", "", worldFrame, registry);

   public UpcomingFootstepList(FootstepProvider footstepProvider, YoVariableRegistry parentRegistry)
   {
      this.footstepProvider = footstepProvider;
      parentRegistry.addChild(registry);
   }

   public void checkForFootsteps()
   {
      if (footstepProvider == null && localFootstepList.isEmpty())
         return;

      for (int i = outputFootstepList.size() - 1; i > footstepIndex.getIntegerValue(); i--)
      {
         outputFootstepList.remove(i);
      }

      numberOfLocalFootsteps.set(localFootstepList.size());

      boolean grabNextFootstepFromLocalList = !localFootstepList.isEmpty();

      Footstep nextFootstep;
      if (grabNextFootstepFromLocalList)
         nextFootstep = localFootstepList.remove(0);
      else
         nextFootstep = footstepProvider.poll();

      if (nextFootstep != null)
      {
         outputFootstepList.add(nextFootstep);
         footstepIndex.set(outputFootstepList.size() - 1);

         FramePose pose = new FramePose();
         nextFootstep.getPose(pose);
         nextFootstepPose.set(pose);

         Footstep nextNextFootstep = null;
         Footstep nextNextNextFootstep = null;

         if (!grabNextFootstepFromLocalList || localFootstepList.isEmpty())
         {
            nextNextFootstep = footstepProvider.peek();
            nextNextNextFootstep = footstepProvider.peekPeek();
         }
         else if (localFootstepList.size() == 1)
         {
            nextNextFootstep = localFootstepList.get(0);
            nextNextNextFootstep = footstepProvider.peek();
         }
         else if (localFootstepList.size() >= 2)
         {
            nextNextFootstep = localFootstepList.get(0);
            nextNextFootstep = localFootstepList.get(1);
         }

         if (nextNextFootstep != null)
            outputFootstepList.add(nextNextFootstep);

         if (nextNextNextFootstep != null)
            outputFootstepList.add(nextNextNextFootstep);
      }
      else
      {
         outputFootstepList.clear();
         footstepIndex.set(0);
      }
   }

   public void requestCancelPlanToProvider()
   {
      footstepProvider.cancelPlan();
   }

   public void clearCurrentFootsteps()
   {
      outputFootstepList.clear();
      footstepIndex.set(0);
      clearLocalFootstepOnly();
   }

   public void clearLocalFootstepOnly()
   {
      localFootstepList.clear();
   }

   public void insertNewNextFootstep(Footstep newNextFootstep)
   {
      if (newNextFootstep != null)
         localFootstepList.add(0, newNextFootstep);
   }

   public Footstep getNextFootstep()
   {
      if (footstepIndex.getIntegerValue() >= outputFootstepList.size())
         return null;
      Footstep nextFootstep = outputFootstepList.get(footstepIndex.getIntegerValue());

      return nextFootstep;
   }

   public Footstep getNextNextFootstep()
   {
      if (footstepIndex.getIntegerValue() + 1 >= outputFootstepList.size())
         return null;
      Footstep nextNextFootstep = outputFootstepList.get(footstepIndex.getIntegerValue() + 1);

      return nextNextFootstep;
   }

   public Footstep getNextNextNextFootstep()
   {
      if (footstepIndex.getIntegerValue() + 2 >= outputFootstepList.size())
         return null;
      Footstep nextNextNextFootstep = outputFootstepList.get(footstepIndex.getIntegerValue() + 2);

      return nextNextNextFootstep;
   }

   public void notifyComplete(FramePose acutalFootPoseInWorld)
   {
      if (footstepProvider == null)
         return;

      footstepProvider.notifyComplete(acutalFootPoseInWorld);
   }

   public void notifyWalkingComplete()
   {
      if (footstepProvider == null)
         return;

      footstepProvider.notifyWalkingComplete();
   }

   public boolean isFootstepProviderEmpty()
   {
      if (footstepProvider == null)
         return true;

      return footstepProvider.isEmpty();
   }

   public int getNumberOfFootstepsToProvide()
   {
      if (footstepProvider == null)
         return 0;

      return footstepProvider.getNumberOfFootstepsToProvide();
   }

   public boolean doesNextFootstepListHaveFewerThanTwoElements()
   {
      return (footstepIndex.getIntegerValue() < 2);
   }

   public Footstep getFootstepTwoBackFromNextFootstepList()
   {
      return outputFootstepList.get(footstepIndex.getIntegerValue() - 2);
   }

   public boolean hasNextFootsteps()
   {
      return (outputFootstepList.size() > 0);
   }

   public boolean isPaused()
   {
      if (footstepProvider == null)
         return false;

      return footstepProvider.isPaused();
   }
}