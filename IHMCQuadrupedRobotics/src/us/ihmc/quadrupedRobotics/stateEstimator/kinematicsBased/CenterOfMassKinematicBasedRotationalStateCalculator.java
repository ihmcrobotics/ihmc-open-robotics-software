package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJointReferenceFrame;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class CenterOfMassKinematicBasedRotationalStateCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private SixDoFJointReferenceFrame rootJointFrame;

   private final YoFrameQuaternion rootJointYoOrientation = new YoFrameQuaternion("estimatedRootJointOrientationWithKinematics", worldFrame, registry);
   private final FrameOrientation rootJointOrientation = new FrameOrientation(worldFrame);

   private final YoFrameQuaternion previousYoRootJointOrientation = new YoFrameQuaternion("previousEstimatedRootJointOrientation", worldFrame, registry);
   
   private final RootJointOrientationCorrectorHelper rootJointOrientationCorrectorHelper = new RootJointOrientationCorrectorHelper("stateEstimator", registry);

   private final ArrayList<RobotQuadrant> allQuadrants = new ArrayList<>();

   private RobotQuadrant usedQuadrant = RobotQuadrant.HIND_LEFT;
   private final EnumYoVariable<RobotQuadrant> yoUsedQuadrant = new EnumYoVariable<RobotQuadrant>("usedQuadrantForOrientationEstimation", registry,
         RobotQuadrant.class, true);

   private final QuadrupedErrorCalculatorBasedOnThreePointsAverageReference errorCalculator;
   private final FrameOrientation orientationError = new FrameOrientation();

   private final FeetPositionCalculator feetPositionCalculator;

   private final QuadrantDependentList<YoFramePoint> calculatedYoFeetPositions = new QuadrantDependentList<>();

   private final QuadrantDependentList<FramePoint> estimatedFeetPositions = new QuadrantDependentList<>();
   private final QuadrantDependentList<FramePoint> calculatedFeetPositions = new QuadrantDependentList<>();

   public CenterOfMassKinematicBasedRotationalStateCalculator(FullInverseDynamicsStructure inverseDynamicsStructure,
         QuadrantDependentList<RigidBody> shinRigidBodies, QuadrantDependentList<ReferenceFrame> footFrames,
         QuadrantDependentList<YoFramePoint> yoEstimatedFootPositions, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.rootJointFrame = inverseDynamicsStructure.getRootJoint().getFrameAfterJoint();

      for (RobotQuadrant quadrant : RobotQuadrant.values)
         allQuadrants.add(quadrant);

      errorCalculator = new QuadrupedErrorCalculatorBasedOnThreePointsAverageReference(registry, graphicsListRegistry);

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

   public void initialize(FrameOrientation comInitialOrientation)
   {
      rootJointOrientationCorrectorHelper.setCorrectionAlpha(0.1);

      rootJointYoOrientation.set(comInitialOrientation);

      feetPositionCalculator.initialize();

      yoUsedQuadrant.set(usedQuadrant);
   }

   public void estimateCoMOrientation(ArrayList<RobotQuadrant> feetInContact, ArrayList<RobotQuadrant> feetNotInContact,
         QuadrantDependentList<YoFramePoint> estimatedYoFeetPositions, YoFramePoint previousRootJointPosition)
   {
      feetPositionCalculator.updateFootToRootJointPositions();

      //compare orientation
      if (feetNotInContact.size() == 0)
      {
         //estimate feet position with the previous estimate and the current joint angles, and store them in an array of calculated feet position
         feetPositionCalculator.estimateFeetPosition(previousRootJointPosition, previousYoRootJointOrientation, feetInContact, calculatedYoFeetPositions);
         
         copyYoFeetPositionsInFeetPositions(estimatedYoFeetPositions);
         
         errorCalculator.updateReferenceFrames(estimatedFeetPositions, calculatedFeetPositions);
         errorCalculator.getOrientationError(usedQuadrant, orientationError);
         compensateOrientationError(previousYoRootJointOrientation);
      }
      else if (feetNotInContact.size() == 1)
      {
         usedQuadrant = feetNotInContact.get(0);
         yoUsedQuadrant.set(usedQuadrant);

         //estimate feet position with the previous estimate and the current joint angles, and store them in an array of calculated feet position
         feetPositionCalculator.estimateFeetPosition(previousRootJointPosition, previousYoRootJointOrientation, feetInContact, calculatedYoFeetPositions);
         copyYoFeetPositionsInFeetPositions(estimatedYoFeetPositions);
         errorCalculator.updateReferenceFrames(estimatedFeetPositions, calculatedFeetPositions);
         errorCalculator.getOrientationError(usedQuadrant, orientationError);
         compensateOrientationError(previousYoRootJointOrientation);
      }
      else
      {
         rootJointYoOrientation.set(previousYoRootJointOrientation);
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

   private void compensateOrientationError(YoFrameQuaternion previousRootJointOrientation)
   {
      rootJointOrientationCorrectorHelper.compensateOrientationError(rootJointOrientation, orientationError);
      rootJointYoOrientation.set(rootJointOrientation);
      previousRootJointOrientation.set(rootJointOrientation);
   }

   public void getCoMOrientation(FrameOrientation orientationToPack)
   {
      rootJointYoOrientation.getFrameOrientationIncludingFrame(orientationToPack);
   }
}
