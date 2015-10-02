package us.ihmc.darpaRoboticsChallenge.footstepGenerator;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.trajectories.SwingTrajectoryHeightCalculator;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepData;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.FootstepOverheadPath;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.FootstepValidityMetric;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.FootstepSnapper;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Created by agrabertilton on 3/31/15.
 */
public class MultiSegmentPathFootstepGenerator extends PathToFootstepGenerator
{
   private ArrayList<FootstepOverheadPath> overheadPathList = new ArrayList<>();
   private SideDependentList<FootstepData> initialFeet = new SideDependentList<>();

   public MultiSegmentPathFootstepGenerator(SwingTrajectoryHeightCalculator heightCalculator, FootstepSnapper footstepSnapper, FootstepValidityMetric validityMetric, HeightMapWithPoints heightMap)
   {
      super(heightCalculator, footstepSnapper, validityMetric, heightMap);
   }

   @Override
   public void initialize(SideDependentList<FootstepData> originalFeet, FootstepOverheadPath overheadPath)
   {
      overheadPathList.clear();
      overheadPathList.add(overheadPath);
      initialFeet = originalFeet;
      super.initialize(originalFeet, overheadPath);
   }

   public void addSegment(FootstepOverheadPath nextPath){
      if (!overheadPathList.isEmpty()){
         overheadPathList.get(0).checkReferenceFrameMatch(nextPath);
      }
      overheadPathList.add(nextPath);
   }

   @Override
   public ArrayList<FootstepData> getStepsAlongPath(RobotSide firstStepSide)
   {

      if (overheadPathList.isEmpty())
      {
         throw new RuntimeException(this.getClass().getSimpleName() + ": No Path for generator");
      }
//      if (!hasHeightMap())
//      {
//         TODO: @Agraber do simple planning assuming flat ground at the initial footstep height
//         throw new RuntimeException(this.getClass().getSimpleName() + ": No HeightMap");
//      }

      ArrayList<FootstepData> generatedSteps = new ArrayList<>();

      RobotSide lastStepSide = firstStepSide.getOppositeSide();
      SideDependentList<FootstepData> lastFootsteps = initialFeet;

      for (FootstepOverheadPath overheadPath : overheadPathList){
         //for each segment, generate a path to the end, then use that as the initial for the next
         super.initialize(lastFootsteps, overheadPath);
         List<FootstepData> segmentFootsteps = super.getStepsAlongPath(lastStepSide.getOppositeSide());
         for (FootstepData footstep : segmentFootsteps){
            generatedSteps.add(footstep);
            lastStepSide = footstep.getRobotSide();
            lastFootsteps.put(lastStepSide, footstep);
         }
         //check that the segment reached the goal
         FramePose2d segmentGoal = overheadPath.getPoseAtDistance(overheadPath.getTotalDistance());
         if (!isGoalFootstep(lastFootsteps.get(lastStepSide), segmentGoal, horizontalDistance)){
            break;
         }
      }

      return generatedSteps;
   }

   public ArrayList<Footstep> concatenateFootstepPaths(ArrayList<Footstep> firstSetOfSteps, ArrayList<Footstep> secondSetOfSteps)
   {
      int indexOfLastStepOfFirstSegment = firstSetOfSteps.size() - 1;
      RobotSide sideOfLastFootstepOfFirstSegment = firstSetOfSteps.get(indexOfLastStepOfFirstSegment).getRobotSide();
      RobotSide sideOfFirstFootstepOfSecondSegment = secondSetOfSteps.get(0).getRobotSide();

      if (sideOfLastFootstepOfFirstSegment == sideOfFirstFootstepOfSecondSegment)
         firstSetOfSteps.remove(indexOfLastStepOfFirstSegment);

      for (Footstep footstep : secondSetOfSteps)
      {
         firstSetOfSteps.add(footstep);
      }

      return firstSetOfSteps;
   }
}
