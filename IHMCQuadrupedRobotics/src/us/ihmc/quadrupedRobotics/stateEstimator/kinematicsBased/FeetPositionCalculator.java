package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import java.util.ArrayList;

import javax.vecmath.Quat4d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class FeetPositionCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ReferenceFrame rootJointFrame;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable alphaFootToRootJointPosition = new DoubleYoVariable(getClass().getSimpleName() + "alphaFootToRootJointPosition", registry);
   private final QuadrantDependentList<AlphaFilteredYoFrameVector> footToRootJointPositions = new QuadrantDependentList<AlphaFilteredYoFrameVector>();

   private final QuadrantDependentList<ReferenceFrame> footFrames;

   //Temporary variables
   private final Quat4d tempQuaternion = new Quat4d();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final FramePoint tempFramePoint = new FramePoint();
   private final FrameVector tempFrameVector = new FrameVector();
   private final FramePoint tempRootPosition = new FramePoint();
   private final FrameOrientation tempRootOrientation = new FrameOrientation();

   public FeetPositionCalculator(ReferenceFrame rootJointFrame, QuadrantDependentList<ReferenceFrame> footFrames, YoVariableRegistry parentRegistry)
   {
      this.rootJointFrame = rootJointFrame;
      this.footFrames = footFrames;

      parentRegistry.addChild(registry);
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         String quadrantPrefix = quadrant.getCamelCaseNameForStartOfExpression();
         AlphaFilteredYoFrameVector footToRootJointPosition = AlphaFilteredYoFrameVector
               .createAlphaFilteredYoFrameVector(quadrantPrefix + "FootToRootJointPosition", "", registry, alphaFootToRootJointPosition, worldFrame);
         footToRootJointPositions.put(quadrant, footToRootJointPosition);
      }
   }

   public void initialize()
   {
      alphaFootToRootJointPosition.set(0.0);
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         footFrames.get(quadrant).update();
      updateFootToRootJointPositions();
      alphaFootToRootJointPosition.set(0.15); //XXX: tune me!!!
   }

   public void estimateFeetPosition(ArrayList<RobotQuadrant> feetToBeUpdated, QuadrantDependentList<YoFramePoint> footPositionsInWorldToPack)
   {
      tempRootPosition.setToZero(rootJointFrame);
      tempRootPosition.changeFrame(worldFrame);

      tempRootOrientation.setToZero(rootJointFrame);
      tempRootOrientation.changeFrame(worldFrame);

      for (int i = 0; i < feetToBeUpdated.size(); i++)
      {
         RobotQuadrant footToBeUpdated = feetToBeUpdated.get(i);
         YoFramePoint footPositionInWorld = footPositionsInWorldToPack.get(footToBeUpdated);
         footPositionInWorld.set(footToRootJointPositions.get(footToBeUpdated));
         footPositionInWorld.scale(-1.0);
         footPositionInWorld.add(tempRootPosition);

         tempRootOrientation.getQuaternion(tempQuaternion);
         tempTransform.setRotationAndZeroTranslation(tempQuaternion);
         footPositionInWorld.applyTransform(tempTransform);
      }
   }

   public void updateFootToRootJointPositions()
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         tempFramePoint.setToZero(rootJointFrame);
         tempFramePoint.changeFrame(footFrames.get(quadrant));

         tempFrameVector.setIncludingFrame(tempFramePoint);
         tempFrameVector.changeFrame(worldFrame);

         footToRootJointPositions.get(quadrant).update(tempFrameVector);
      }
   }

   public void estimateFeetPosition(YoFramePoint previousRootJointPosition, YoFrameQuaternion previousRootJointYoOrientation,
         ArrayList<RobotQuadrant> feetToBeUpdated, QuadrantDependentList<YoFramePoint> footPositionsInWorldToPack)
   {
      for (int i = 0; i < feetToBeUpdated.size(); i++)
      {
         RobotQuadrant footToBeUpdated = feetToBeUpdated.get(i);
         YoFramePoint footPositionInWorld = footPositionsInWorldToPack.get(footToBeUpdated);
         footPositionInWorld.set(footToRootJointPositions.get(footToBeUpdated));
         footPositionInWorld.scale(-1.0);
         footPositionInWorld.add(previousRootJointPosition);

         previousRootJointYoOrientation.get(tempQuaternion);
         tempTransform.setRotationAndZeroTranslation(tempQuaternion);
         footPositionInWorld.applyTransform(tempTransform);
      }
      
   }
}
