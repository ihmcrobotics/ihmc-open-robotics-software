package us.ihmc.commonWalkingControlModules.configurations;

import java.util.LinkedHashMap;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotSide.SideDependentList;

public class SliderBoardParameters
{
   public boolean controlHeadAndHandsWithSliders()
   {
      return false;
   }

   public SideDependentList<LinkedHashMap<String, ImmutablePair<Double, Double>>> getSliderBoardControlledFingerJointsWithLimits()
   {
      return new SideDependentList<LinkedHashMap<String, ImmutablePair<Double,Double>>>();
   }

   public LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>> getSliderBoardControlledNeckJointsWithLimits()
   {
      return new LinkedHashMap<NeckJointName, ImmutablePair<Double,Double>>();
   }
}
