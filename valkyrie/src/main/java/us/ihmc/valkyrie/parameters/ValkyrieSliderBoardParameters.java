package us.ihmc.valkyrie.parameters;

import java.util.LinkedHashMap;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.commonWalkingControlModules.configurations.SliderBoardParameters;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.valkyrie.configuration.ValkyrieSliderBoardControlledNeckJoints;

public class ValkyrieSliderBoardParameters extends SliderBoardParameters
{
   private final LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>> sliderBoardControlledNeckJointNamesWithLimits = new LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>>();
   private final SideDependentList<LinkedHashMap<String, ImmutablePair<Double, Double>>> sliderBoardControlledFingerJointNamesWithLimits = new SideDependentList<LinkedHashMap<String, ImmutablePair<Double, Double>>>();

   public ValkyrieSliderBoardParameters()
   {
      for (RobotSide side : RobotSide.values())
      {
         sliderBoardControlledFingerJointNamesWithLimits.put(side, new LinkedHashMap<String, ImmutablePair<Double,Double>>());
         // FIXME 
//         for(ValkyrieRealRobotFingerJoint fingerJoint : ValkyrieRealRobotFingerJoint.values())
//         {
//            sliderBoardControlledFingerJointNamesWithLimits.get(side).put(side.getCamelCaseNameForStartOfExpression() + fingerJoint.toString(),
//                  new ImmutablePair<Double,Double>(ValkyrieFingerJointLimits.getFullyExtensonPositionLimit(side, fingerJoint), ValkyrieFingerJointLimits.getFullyFlexedPositionLimit(side, fingerJoint)));
//         }
      }

      NeckJointName[] sliderBoardControlledNeckJointNames = ValkyrieSliderBoardControlledNeckJoints.getNeckJointsControlledBySliderBoard();

      for (int i = 0; i < sliderBoardControlledNeckJointNames.length; i++)
      {
         NeckJointName joint = sliderBoardControlledNeckJointNames[i];

         sliderBoardControlledNeckJointNamesWithLimits.put(
               joint,
               new ImmutablePair<Double, Double>(ValkyrieSliderBoardControlledNeckJoints.getFullyExtendedPositionLimit(joint), ValkyrieSliderBoardControlledNeckJoints
                     .getFullyFlexedPositionLimit(joint)));
      }
   }

   @Override
   public LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>> getSliderBoardControlledNeckJointsWithLimits()
   {
      return sliderBoardControlledNeckJointNamesWithLimits;
   }

   @Override
   public SideDependentList<LinkedHashMap<String, ImmutablePair<Double, Double>>> getSliderBoardControlledFingerJointsWithLimits()
   {
      return sliderBoardControlledFingerJointNamesWithLimits;
   }

   @Override
   public boolean controlHeadAndHandsWithSliders()
   {
      return false;
   }
}
