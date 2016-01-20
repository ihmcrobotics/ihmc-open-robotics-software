package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import java.util.ArrayList;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

/*
 * This class does not work
 */

public class QuadrupedErrorCalculatorBasedOnThreePointsAverageReference
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final QuadrantDependentList<FramePoint> referenceFeetPositions = new QuadrantDependentList<>();
   private final QuadrantDependentList<ThreePointsAverageReferenceFrame> referenceFeetAverageReferenceFrames = new QuadrantDependentList<>();

   private final QuadrantDependentList<FramePoint> comparedFeetPositions = new QuadrantDependentList<>();
   private final QuadrantDependentList<ThreePointsAverageReferenceFrame> comparedFeetAverageReferenceFrames = new QuadrantDependentList<>();

   //Visualization Variables
  private final ArrayList<YoGraphicReferenceFrame> referenceFeetAverageReferenceFrameVizs = new ArrayList<>();
  private final ArrayList<YoGraphicReferenceFrame> comparedFeetAverageReferenceFrameVizs = new ArrayList<>();
   
   public QuadrupedErrorCalculatorBasedOnThreePointsAverageReference(String name, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         FramePoint referenceFootPosition = new FramePoint(worldFrame);
         referenceFeetPositions.put(quadrant, referenceFootPosition);

         FramePoint comparedFootPosition = new FramePoint(worldFrame);
         comparedFeetPositions.put(quadrant, comparedFootPosition);
      }
      
      //this one needs to be in a difference for loop because all the feetPosition FramePoints have to be previously created
      constructAverageFrame(RobotQuadrant.FRONT_LEFT, RobotQuadrant.FRONT_RIGHT, RobotQuadrant.HIND_RIGHT, RobotQuadrant.HIND_LEFT, name, graphicsListRegistry);
      constructAverageFrame(RobotQuadrant.FRONT_RIGHT, RobotQuadrant.HIND_RIGHT, RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_LEFT, name, graphicsListRegistry);
      constructAverageFrame(RobotQuadrant.HIND_RIGHT, RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_LEFT, RobotQuadrant.FRONT_RIGHT, name, graphicsListRegistry);
      constructAverageFrame(RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_LEFT, RobotQuadrant.FRONT_RIGHT, RobotQuadrant.HIND_RIGHT, name, graphicsListRegistry);
   
      parentRegistry.addChild(registry);
   }

   //The quadrantNotUsed is the key of the quadrantDependentLists containing the ThreePointsAverageReferenceFrame
   private void constructAverageFrame(RobotQuadrant quadrant1, RobotQuadrant quadrant2, RobotQuadrant quadrant3, RobotQuadrant quadrantNotUsed,
         String name, YoGraphicsListRegistry graphicsListRegistry)
   {
      String prefix = name + quadrantNotUsed.getCamelCaseNameForMiddleOfExpression();

      //construction of the estimated related variables
      FramePoint referenceP1 = referenceFeetPositions.get(quadrant1);
      FramePoint referenceP2 = referenceFeetPositions.get(quadrant2);
      FramePoint referenceP3 = referenceFeetPositions.get(quadrant3);

      ThreePointsAverageReferenceFrame estimatedFootAverageReferenceFrame = new ThreePointsAverageReferenceFrame(prefix + "EstimatedFootAverageReferenceFrame",
            referenceP1, referenceP2, referenceP3, worldFrame);
      referenceFeetAverageReferenceFrames.put(quadrantNotUsed, estimatedFootAverageReferenceFrame);

      YoGraphicReferenceFrame estimatedFootAverageRefViz = new YoGraphicReferenceFrame(estimatedFootAverageReferenceFrame, registry, 0.15,
            YoAppearance.Yellow());
      graphicsListRegistry.registerYoGraphic(prefix + "KinematicsBasedStateEstimatorEstimated", estimatedFootAverageRefViz);
      referenceFeetAverageReferenceFrameVizs.add(estimatedFootAverageRefViz);

      //construction of the calculated related variables
      FramePoint comparedP1 = comparedFeetPositions.get(quadrant1);
      FramePoint comparedP2 = comparedFeetPositions.get(quadrant2);
      FramePoint comparedP3 = comparedFeetPositions.get(quadrant3);

      ThreePointsAverageReferenceFrame calculatedFootAverageReferenceFrame = new ThreePointsAverageReferenceFrame(
            prefix + "CalculatedFootAverageReferenceFrame", comparedP1, comparedP2, comparedP3, worldFrame);
      comparedFeetAverageReferenceFrames.put(quadrantNotUsed, calculatedFootAverageReferenceFrame);

      YoGraphicReferenceFrame calculatedFootAverageRefViz = new YoGraphicReferenceFrame(calculatedFootAverageReferenceFrame, registry, 0.2,
            YoAppearance.Green());
      graphicsListRegistry.registerYoGraphic(prefix + "KinematicsBasedStateEstimatorCalculated", calculatedFootAverageRefViz);
      comparedFeetAverageReferenceFrameVizs.add(calculatedFootAverageRefViz);
   }

   public void updateReferenceFrames(QuadrantDependentList<FramePoint> referenceFeetPositions, QuadrantDependentList<FramePoint> comparedFeetPositions)
   {
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         this.referenceFeetPositions.get(quadrant).set(referenceFeetPositions.get(quadrant));
         this.comparedFeetPositions.get(quadrant).set(comparedFeetPositions.get(quadrant));
      }
      
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         referenceFeetAverageReferenceFrames.get(quadrant).update();
         comparedFeetAverageReferenceFrames.get(quadrant).update();
      }
   }
   
   public void getPositionError(RobotQuadrant quadrantToBeUsed, FramePoint positionErrorToPack)
   {
      positionErrorToPack.setToZero(referenceFeetAverageReferenceFrames.get(quadrantToBeUsed));
      positionErrorToPack.changeFrame(comparedFeetAverageReferenceFrames.get(quadrantToBeUsed));
   }

   public void getOrientationError(RobotQuadrant quadrantToBeUsed, FrameOrientation orientationErrorToPack)
   {
      orientationErrorToPack.setToZero(referenceFeetAverageReferenceFrames.get(quadrantToBeUsed));
      orientationErrorToPack.changeFrame(comparedFeetAverageReferenceFrames.get(quadrantToBeUsed));
   }
   
   public void getPoseError(RobotQuadrant quadrantToBeUsed, FramePose poseErrorToPack)
   {
      
   }
}
