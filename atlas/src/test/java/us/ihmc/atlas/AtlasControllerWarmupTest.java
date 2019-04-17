package us.ihmc.atlas;

import java.util.Arrays;
import java.util.Collection;
import java.util.EnumMap;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableInt;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.warmup.HumanoidControllerWarumupTools;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.robotics.Assert;
import us.ihmc.yoVariables.variable.YoEnum;

public class AtlasControllerWarmupTest
{
   @Test
   @SuppressWarnings("unchecked")
   public void testWarmup()
   {
      AtlasControllerWarmup controllerWarmup = new AtlasControllerWarmup();

      YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) controllerWarmup.getYoVariable("WalkingCurrentState");
      Map<WalkingStateEnum, MutableInt> ticksSpentInStateMap = new EnumMap<>(WalkingStateEnum.class);
      for (WalkingStateEnum walkingStateEnum : WalkingStateEnum.values())
      {
         ticksSpentInStateMap.put(walkingStateEnum, new MutableInt(0));
      }

      controllerWarmup.addTickListener(() -> {
         WalkingStateEnum currentWalkingState = walkingState.getEnumValue();
         ticksSpentInStateMap.get(currentWalkingState).increment();
      });

      HumanoidControllerWarumupTools.warmup(controllerWarmup);

      int minTicksInEachState = 200;
      for (WalkingStateEnum stateToCheck : getWalkingStatesToCheck())
      {
         Assert.assertTrue("Did not spend enough time in walking state " + stateToCheck, ticksSpentInStateMap.get(stateToCheck).intValue() > minTicksInEachState);
      }
   }

   private Collection<WalkingStateEnum> getWalkingStatesToCheck()
   {
      return Arrays.asList(new WalkingStateEnum[] {WalkingStateEnum.STANDING, WalkingStateEnum.TO_STANDING, WalkingStateEnum.TO_WALKING_LEFT_SUPPORT,
            WalkingStateEnum.TO_WALKING_RIGHT_SUPPORT, WalkingStateEnum.WALKING_LEFT_SUPPORT, WalkingStateEnum.WALKING_RIGHT_SUPPORT});
   }
}
