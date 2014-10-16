package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.humanoidRobot.footstep.Footstep;
import us.ihmc.yoUtilities.math.frames.YoFramePose;


public class UpcomingFootstepList
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FootstepProvider footstepProvider;

   private final IntegerYoVariable nextFootstepIndex = new IntegerYoVariable("nextFootstepIndex", registry);
   private final IntegerYoVariable nextNextFootstepIndex = new IntegerYoVariable("nextNextFootstepIndex", registry);
   private final IntegerYoVariable nextNextNextFootstepIndex = new IntegerYoVariable("nextNextNextFootstepIndex", registry);

   private final List<Footstep> nextFootstepList = new ArrayList<Footstep>();
   private final ArrayList<Footstep> nextNextFootstepList = new ArrayList<Footstep>();
   private final ArrayList<Footstep> nextNextNextFootstepList = new ArrayList<Footstep>();

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFramePose nextFootstepPose = new YoFramePose("nextFootstep", "", worldFrame, registry);

   public UpcomingFootstepList(FootstepProvider footstepProvider, YoVariableRegistry parentRegistry)
   {
      this.footstepProvider = footstepProvider;
      parentRegistry.addChild(registry);
   }

   public void checkForFootsteps(BooleanYoVariable readyToGrabNextFootstep, EnumYoVariable<RobotSide> upcomingSupportLeg,
         SideDependentList<? extends ContactablePlaneBody> bipedFeet)
   {
      if (footstepProvider == null)
         return;

      if (readyToGrabNextFootstep.getBooleanValue())
      {
         for (int i = nextFootstepList.size() - 1; i > nextFootstepIndex.getIntegerValue(); i--)
         {
            nextFootstepList.remove(i);
         }

         for (int i = nextNextFootstepList.size() - 1; i > nextNextFootstepIndex.getIntegerValue(); i--)
         {
            nextNextFootstepList.remove(i);
         }

         for (int i = nextNextNextFootstepList.size() - 1; i > nextNextNextFootstepIndex.getIntegerValue(); i--)
         {
            nextNextNextFootstepList.remove(i);
         }

         Footstep nextFootstep = footstepProvider.poll();

         if (nextFootstep != null)
         {
            nextFootstepList.add(nextFootstep);
            nextFootstepIndex.set(nextFootstepList.size() - 1);

            upcomingSupportLeg.set(nextFootstep.getRobotSide().getOppositeSide());
            FramePose pose = new FramePose();
            nextFootstep.getPose(pose);
            nextFootstepPose.set(pose);

            readyToGrabNextFootstep.set(false);

            Footstep nextNextFootstep = footstepProvider.peek();
            if (nextNextFootstep != null)
            {
               nextNextFootstepList.add(nextNextFootstep);
               nextNextFootstepIndex.set(nextNextFootstepList.size() - 1);
            }
            else
            {
               nextNextFootstepIndex.increment();
            }

            Footstep nextNextNextFootstep = footstepProvider.peekPeek();
            if (nextNextNextFootstep != null)
            {
               nextNextNextFootstepList.add(nextNextNextFootstep);
               nextNextNextFootstepIndex.set(nextNextNextFootstepList.size() - 1);
            }
            else
            {
               nextNextNextFootstepIndex.increment();
            }
         }

         else
         {
            nextFootstepList.clear();
            nextFootstepIndex.set(0);
            nextNextFootstepList.clear();
            nextNextFootstepIndex.set(0);
            nextNextNextFootstepList.clear();
            nextNextNextFootstepIndex.set(0);
         }
      }
   }

   public Footstep getNextFootstep()
   {
      if (nextFootstepIndex.getIntegerValue() >= nextFootstepList.size())
         return null;
      Footstep nextFootstep = nextFootstepList.get(nextFootstepIndex.getIntegerValue());

      return nextFootstep;
   }

   public Footstep getNextNextFootstep()
   {
      if (nextNextFootstepIndex.getIntegerValue() >= nextNextFootstepList.size())
         return null;
      Footstep nextNextFootstep = nextNextFootstepList.get(nextNextFootstepIndex.getIntegerValue());

      return nextNextFootstep;
   }

   public Footstep getNextNextNextFootstep()
   {
      if (nextNextNextFootstepIndex.getIntegerValue() >= nextNextNextFootstepList.size())
         return null;
      Footstep nextNextNextFootstep = nextNextNextFootstepList.get(nextNextNextFootstepIndex.getIntegerValue());

      return nextNextNextFootstep;
   }

   public void notifyComplete()
   {
      if (footstepProvider == null)
         return;

      footstepProvider.notifyComplete();
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
      return (nextFootstepIndex.getIntegerValue() < 2);
   }

   public Footstep getFootstepTwoBackFromNextFootstepList()
   {
      return nextFootstepList.get(nextFootstepIndex.getIntegerValue() - 2);
   }

   public boolean hasNextFootsteps()
   {
      return (nextFootstepList.size() > 0);
   }
}