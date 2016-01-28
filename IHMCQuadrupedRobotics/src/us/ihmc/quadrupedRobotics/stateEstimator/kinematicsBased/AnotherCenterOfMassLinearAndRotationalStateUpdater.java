package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import java.util.ArrayList;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class AnotherCenterOfMassLinearAndRotationalStateUpdater
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SixDoFJoint rootJoint;
   private final ReferenceFrame rootJointFrame;

   private final YoFramePoint yoRootJointPosition = new YoFramePoint("AnotherEstimatedRootJointPosition", worldFrame, registry);
   private final YoFrameQuaternion yoRootJointOrientation = new YoFrameQuaternion("AnotherEstimatedRootJointOrientation", worldFrame, registry);

   private final AnotherCenterOfMassKinematicBasedLinearAndRotationalStateCalculator comCalculator;

   private final FeetPositionCalculator feetPositionCalculator;
   private final QuadrantDependentList<YoFramePoint> estimatedYoFeetPositions = new QuadrantDependentList<>();

   private double initialHeight = 0.0;

   private final ArrayList<RobotQuadrant> allQuadrants = new ArrayList<>();

   // Temporary variables
   private final FramePoint rootJointPosition = new FramePoint(worldFrame);
   private final FrameOrientation rootJointOrientation = new FrameOrientation(worldFrame);
   private final Vector3d tempRootJointTranslation = new Vector3d();
   private final Quat4d tempRootJointOrientation = new Quat4d(0.0, 0.0, 0.0, 1.0);

   public AnotherCenterOfMassLinearAndRotationalStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure,
         QuadrantDependentList<ReferenceFrame> footFrames, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      rootJoint = inverseDynamicsStructure.getRootJoint();
      rootJointFrame = rootJoint.getFrameAfterJoint();

      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         YoFramePoint yoFootPosition = new YoFramePoint(quadrant.getCamelCaseNameForStartOfExpression() + "AnotherEstimatedFootPositionInWorld", worldFrame, registry);
         estimatedYoFeetPositions.set(quadrant, yoFootPosition);

         YoGraphicPosition footPositionViz = new YoGraphicPosition(quadrant.getCamelCaseNameForStartOfExpression() + "AnotherEstimatedPosition", yoFootPosition, 0.02,
               YoAppearance.Yellow());
         graphicsListRegistry.registerYoGraphic("KinematicsBasedStateEstimatorEstimated", footPositionViz);
      }

      comCalculator = new AnotherCenterOfMassKinematicBasedLinearAndRotationalStateCalculator(inverseDynamicsStructure, footFrames, registry,
            graphicsListRegistry);

      feetPositionCalculator = new FeetPositionCalculator(rootJointFrame, footFrames, registry);

      for (RobotQuadrant quadrant : RobotQuadrant.values)
         allQuadrants.add(quadrant);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      //initialize the CoM to be at 0.0 0.0 0.0 in worldFrame
      //XXX: will need to do initialize that more smartly
//      comCalculator.initialize(new FrameOrientation(worldFrame, 0.0, 0.0, 0.0), new FramePoint(worldFrame, 0.0, 0.0, initialHeight));
      comCalculator.initialize(new FrameOrientation(worldFrame, 0.0, 0.0, 0.0), new FramePoint(worldFrame, 0.01176, -0.00025, 0.54171));
      updateRootJoint();
      feetPositionCalculator.initialize();
      feetPositionCalculator.estimateFeetPosition(yoRootJointPosition, yoRootJointOrientation, allQuadrants, estimatedYoFeetPositions);
   }

   public void updateCenterOfMassLinearAndRotationalState(ArrayList<RobotQuadrant> feetInContact, ArrayList<RobotQuadrant> feetNotInContact)
   {
      comCalculator.estimateCoMPose(feetInContact, feetNotInContact, estimatedYoFeetPositions, yoRootJointPosition);
      
      updateRootJoint();

      feetPositionCalculator.updateFootToRootJointPositions();
      feetPositionCalculator.estimateFeetPosition(yoRootJointPosition, yoRootJointOrientation, feetNotInContact, estimatedYoFeetPositions);
   }

   private void updateRootJoint()
   {
      comCalculator.getCoMPosition(rootJointPosition);
      yoRootJointPosition.set(rootJointPosition);
      rootJointPosition.get(tempRootJointTranslation);
      rootJoint.setPosition(tempRootJointTranslation);

      comCalculator.getCoMOrientation(rootJointOrientation);
      yoRootJointOrientation.set(rootJointOrientation);
      rootJointOrientation.getQuaternion(tempRootJointOrientation);
      rootJoint.setRotation(tempRootJointOrientation);

      rootJointFrame.update();
   }

   public void setInitialHeight(double initialHeight)
   {
      this.initialHeight = initialHeight;
   }
}
