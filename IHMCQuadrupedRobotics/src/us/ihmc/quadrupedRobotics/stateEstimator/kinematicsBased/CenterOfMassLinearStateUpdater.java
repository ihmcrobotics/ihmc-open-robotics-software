package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class CenterOfMassLinearStateUpdater
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FramePoint comPosition = new FramePoint(worldFrame);

   private final SixDoFJoint rootJoint;
   private final ReferenceFrame rootJointFrame;
   private Twist rootJointTwist = new Twist();
   private final TwistCalculator twistCalculator;

   private final QuadrantDependentList<YoFramePoint> footPositions = new QuadrantDependentList<>();

   private final CenterOfMassKinematicBasedCalculator comAndFeetCalculator;

   private final YoFramePoint yoRootJointPosition = new YoFramePoint("estimatedRootJointPosition", worldFrame, registry);
   private final YoFrameVector yoRootJointVelocity = new YoFrameVector("estimatedRootJointVelocity", worldFrame, registry);

   private double initialHeight = 0.0;

   //visualization variables
   private final ArrayList<YoGraphicPosition> feetPositionsVizs = new ArrayList<>();

   // Temporary variables
   private final FramePoint rootJointPosition = new FramePoint(worldFrame);
   private final FrameVector rootJointVelocity = new FrameVector(worldFrame);
   private final Vector3d tempRootJointTranslation = new Vector3d();
   private final FrameVector tempVelocity = new FrameVector();

   public CenterOfMassLinearStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure, QuadrantDependentList<RigidBody> shinRigidBodies,
         QuadrantDependentList<ReferenceFrame> footFrames, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      twistCalculator = inverseDynamicsStructure.getTwistCalculator();

      rootJoint = inverseDynamicsStructure.getRootJoint();
      rootJointFrame = rootJoint.getFrameAfterJoint();

      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         YoFramePoint footPosition = new YoFramePoint(quadrant.getCamelCaseNameForStartOfExpression() + "EstimatedFootPositionInWorld", worldFrame, registry);
         footPositions.set(quadrant, footPosition);

         YoGraphicPosition footPositionViz = new YoGraphicPosition(quadrant.getCamelCaseNameForStartOfExpression() + "EstimatedPosition", footPosition, 0.02,
               YoAppearance.Yellow());
         feetPositionsVizs.add(footPositionViz);
         graphicsListRegistry.registerYoGraphic("KinematicsBasedStateEstimator", footPositionViz);
      }

      comAndFeetCalculator = new CenterOfMassKinematicBasedCalculator(inverseDynamicsStructure, shinRigidBodies, footFrames, registry);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      //initialize the CoM to be at 0.0 0.0 0.0 in worldFrame
      //XXX: will need to do initialize that more smartly
      comAndFeetCalculator.initialize(new FramePoint(worldFrame, 0.0, 0.0, initialHeight), footPositions);
      comAndFeetCalculator.getRootJointPositionAndVelocity(rootJointPosition, rootJointVelocity);
      updateRootJoint();
      updateViz();
   }

   public void updateCenterOfMassLinearState(ArrayList<RobotQuadrant> feetInContact, ArrayList<RobotQuadrant> feetNotInContact)
   {
      comAndFeetCalculator.estimateFeetAndComPosition(feetInContact, feetNotInContact, footPositions, comPosition);
      updateRootJoint();
      updateViz();
   }

   private void updateRootJoint()
   {
      comAndFeetCalculator.getRootJointPositionAndVelocity(rootJointPosition, rootJointVelocity);
      rootJointPosition.get(tempRootJointTranslation);

      yoRootJointPosition.set(rootJointPosition);
      rootJoint.setPosition(tempRootJointTranslation);

      yoRootJointVelocity.set(rootJointVelocity);
      tempVelocity.setIncludingFrame(rootJointVelocity);
      rootJoint.packJointTwist(rootJointTwist);
      tempVelocity.changeFrame(rootJointTwist.getExpressedInFrame());
      rootJointTwist.setLinearPart(tempVelocity);
      rootJoint.setJointTwist(rootJointTwist);

      rootJointFrame.update();
      twistCalculator.compute();

   }

   private void updateViz()
   {
      for (int i = 0; i < feetPositionsVizs.size(); i++)
         feetPositionsVizs.get(i).update();
   }

   public void setInitialHeight(double initialHeight)
   {
      this.initialHeight = initialHeight;
   }
}
