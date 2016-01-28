package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.SixDoFJointReferenceFrame;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class AnotherCenterOfMassKinematicBasedLinearAndRotationalStateCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private SixDoFJointReferenceFrame rootJointFrame;

   private final YoFramePoint rootJointYoPosition = new YoFramePoint("estimatedRootJointPositionWithKinematics", worldFrame, registry);
   private final YoFramePoint previousRootJointYoPosition = new YoFramePoint("previousEstimatedRootJointPositionWithKinematics", worldFrame, registry);
   
   private final YoFrameQuaternion rootJointYoOrientation = new YoFrameQuaternion("estimatedRootJointOrientationWithKinematics", worldFrame, registry);
   private final YoFrameQuaternion previousRootJointYoOrientation = new YoFrameQuaternion("previousEstimatedRootJointOrientation", worldFrame, registry);
   
   private final RootJointPoseCorrectorHelper rootJointPoseCorrectorHelper = new RootJointPoseCorrectorHelper("stateEstimator", registry);
   private final FramePose rootJointPose = new FramePose();

   private final ArrayList<RobotQuadrant> allQuadrants = new ArrayList<>();

   private RobotQuadrant usedQuadrant = RobotQuadrant.HIND_LEFT;
   private final EnumYoVariable<RobotQuadrant> yoUsedQuadrant = new EnumYoVariable<RobotQuadrant>("usedQuadrantForOrientationEstimation", registry,
         RobotQuadrant.class, true);

   private final QuadrupedErrorCalculatorBasedOnThreePointsAverageReference errorCalculator;
   private final FramePose poseError = new FramePose();

   private final FeetPositionCalculator feetPositionCalculator;

   private final QuadrantDependentList<YoFramePoint> calculatedYoFeetPositions = new QuadrantDependentList<>();

   private final QuadrantDependentList<FramePoint> estimatedFeetPositions = new QuadrantDependentList<>();
   private final QuadrantDependentList<FramePoint> calculatedFeetPositions = new QuadrantDependentList<>();

   //Temporary variables
   private FramePoint tempFramePoint = new FramePoint();
   private FrameOrientation tempFrameOrientation = new FrameOrientation();

   public AnotherCenterOfMassKinematicBasedLinearAndRotationalStateCalculator(FullInverseDynamicsStructure inverseDynamicsStructure,
         QuadrantDependentList<ReferenceFrame> footFrames, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry graphicsListRegistry)
   {
      this.rootJointFrame = inverseDynamicsStructure.getRootJoint().getFrameAfterJoint();

      for (RobotQuadrant quadrant : RobotQuadrant.values)
         allQuadrants.add(quadrant);

      errorCalculator = new QuadrupedErrorCalculatorBasedOnThreePointsAverageReference("anotherComLinearAndRotationalCalculator" , registry, graphicsListRegistry);

      feetPositionCalculator = new FeetPositionCalculator(rootJointFrame, footFrames, registry);

      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         YoFramePoint yoFootPosition = new YoFramePoint(quadrant.getCamelCaseNameForStartOfExpression() + "CalculatedFootPositionInWorld", worldFrame,
               registry);
         calculatedYoFeetPositions.set(quadrant, yoFootPosition);

         FramePoint estimatedFootPosition = new FramePoint(worldFrame);
         estimatedFeetPositions.set(quadrant, estimatedFootPosition);

         FramePoint calculatedFootPosition = new FramePoint(worldFrame);
         calculatedFeetPositions.set(quadrant, calculatedFootPosition);
      }
      
      parentRegistry.addChild(registry);
   }

   public void initialize(FrameOrientation comInitialOrientation, FramePoint comInitialPosition)
   {
      rootJointPoseCorrectorHelper.setCorrectionAlpha(0.1, 1.0);

      rootJointPose.setPoseIncludingFrame(comInitialPosition, comInitialOrientation);
      
      rootJointYoOrientation.set(comInitialOrientation);
      rootJointYoPosition.set(comInitialPosition);
      
      previousRootJointYoOrientation.set(comInitialOrientation);
      previousRootJointYoPosition.set(comInitialPosition);
      
      feetPositionCalculator.initialize();

      yoUsedQuadrant.set(usedQuadrant);
   }

   public void estimateCoMPose(ArrayList<RobotQuadrant> feetInContact, ArrayList<RobotQuadrant> feetNotInContact,
         QuadrantDependentList<YoFramePoint> estimatedYoFeetPositions, YoFramePoint previousRootJointPosition)
   {
      feetPositionCalculator.updateFootToRootJointPositions();

      //compare orientation
      if (feetNotInContact.size() == 0)
      {
         //estimate feet position with the previous estimate and the current joint angles, and store them in an array of calculated feet position
         feetPositionCalculator.estimateFeetPosition(previousRootJointPosition, previousRootJointYoOrientation, feetInContact, calculatedYoFeetPositions);
         
         copyYoFeetPositionsInFeetPositions(estimatedYoFeetPositions);
         
         errorCalculator.updateReferenceFrames(estimatedFeetPositions, calculatedFeetPositions);
         errorCalculator.getPoseError(usedQuadrant, poseError);
         compensatePoseError();
      }
      else if (feetNotInContact.size() == 1)
      {
         usedQuadrant = feetNotInContact.get(0);
         yoUsedQuadrant.set(usedQuadrant);

         //estimate feet position with the previous estimate and the current joint angles, and store them in an array of calculated feet position
         feetPositionCalculator.estimateFeetPosition(previousRootJointPosition, previousRootJointYoOrientation, feetInContact, calculatedYoFeetPositions);
         copyYoFeetPositionsInFeetPositions(estimatedYoFeetPositions);
         errorCalculator.updateReferenceFrames(estimatedFeetPositions, calculatedFeetPositions);
         errorCalculator.getPoseError(usedQuadrant, poseError);
         compensatePoseError();
      }
      else
      {
         rootJointYoOrientation.set(previousRootJointYoOrientation);
         rootJointYoPosition.set(previousRootJointYoPosition);
      }
   }

   private void copyYoFeetPositionsInFeetPositions(QuadrantDependentList<YoFramePoint> estimatedYoFeetPositions)
   {
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         estimatedYoFeetPositions.get(quadrant).getFrameTuple(estimatedFeetPositions.get(quadrant));
         calculatedYoFeetPositions.get(quadrant).getFrameTuple(calculatedFeetPositions.get(quadrant));
      }
   }

   private void compensatePoseError()
   {
      rootJointPoseCorrectorHelper.compensatePoseError(rootJointPose, poseError);
      
      rootJointPose.getPoseIncludingFrame(tempFramePoint, tempFrameOrientation);
      
      rootJointYoOrientation.set(tempFrameOrientation);
      previousRootJointYoOrientation.set(rootJointYoOrientation);
      
      
      tempFramePoint.set(0.0118,-0.0003, 0.5417);
      rootJointYoPosition.set(tempFramePoint);
      previousRootJointYoPosition.set(rootJointYoPosition);
   }

   public void getCoMOrientation(FrameOrientation orientationToPack)
   {
      rootJointYoOrientation.getFrameOrientationIncludingFrame(orientationToPack);
   }
 
   public void getCoMPosition(FramePoint positionToPack)
   {
      rootJointYoPosition.getFrameTupleIncludingFrame(positionToPack);
   }
   
   public void getCoMPose(FramePose poseToPack)
   {
      rootJointYoOrientation.getFrameOrientationIncludingFrame(tempFrameOrientation);
      rootJointYoPosition.getFrameTupleIncludingFrame(tempFramePoint);
      
      poseToPack.setPose(tempFramePoint, tempFrameOrientation);
   }
}
