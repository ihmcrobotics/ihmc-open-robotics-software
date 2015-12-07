package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.SixDoFJointReferenceFrame;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class CenterOfMassKinematicBasedCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private SixDoFJoint rootJoint;
   private SixDoFJointReferenceFrame rootJointFrame;
   private TwistCalculator twistCalculator;

   private final YoFramePoint rootJointPosition = new YoFramePoint("estimatedRootJointPositionWithKinematics", worldFrame, registry);
   private final YoFrameVector rootJointLinearVelocityNewTwist = new YoFrameVector("estimatedRootJointVelocityNewTwist", worldFrame, registry);

   private final QuadrantDependentList<Twist> footTwistsInWorld = new QuadrantDependentList<Twist>(new Twist(), new Twist(), new Twist(), new Twist());
   private final QuadrantDependentList<RigidBody> shinRigidBodies;
   private final QuadrantDependentList<ReferenceFrame> footFrames;

   private final YoFrameVector frontLeftFootVelocityInWorld = new YoFrameVector("frontLeftFootVelocityInWorld", worldFrame, registry);
   private final YoFrameVector frontRightFootVelocityInWorld = new YoFrameVector("frontRightFootVelocityInWorld", worldFrame, registry);
   private final YoFrameVector hindLeftFootVelocityInWorld = new YoFrameVector("hindLeftFootVelocityInWorld", worldFrame, registry);
   private final YoFrameVector hindRightFootVelocityInWorld = new YoFrameVector("hindRightFootVelocityInWorld", worldFrame, registry);
   private final QuadrantDependentList<YoFrameVector> footVelocitiesInWorld = new QuadrantDependentList<YoFrameVector>(frontLeftFootVelocityInWorld,
         frontRightFootVelocityInWorld, hindLeftFootVelocityInWorld, hindRightFootVelocityInWorld);

   private final DoubleYoVariable alphaFootToRootJointPosition = new DoubleYoVariable("alphaFootToRootJointPosition", registry);
   private final QuadrantDependentList<AlphaFilteredYoFrameVector> footToRootJointPositions = new QuadrantDependentList<AlphaFilteredYoFrameVector>();

   private final DoubleYoVariable alphaRootJointLinearVelocityNewTwist = new DoubleYoVariable("alphaRootJointLinearVelocityNewTwist", registry);

   private final YoFramePoint previousRootJointPosition = new YoFramePoint("previousEstimatedRootJointPositionWithKinematics", worldFrame, registry);
   private final YoFrameVector previousRootJointLinearVelocityNewTwist = new YoFrameVector("previousEstimatedRootJointVelocityNewTwist", worldFrame, registry);

   //Temporary Variables
   private final Twist tempRootBodyTwist = new Twist();
   private final FrameVector tempFrameVector = new FrameVector();
   private final FramePoint tempFramePoint = new FramePoint();
   private final FramePoint tempPosition = new FramePoint();

   public CenterOfMassKinematicBasedCalculator(FullInverseDynamicsStructure inverseDynamicsStructure, QuadrantDependentList<RigidBody> shinRigidBodies,
         QuadrantDependentList<ReferenceFrame> footFrames, YoVariableRegistry parentRegistry)
   {
      this.rootJoint = inverseDynamicsStructure.getRootJoint();
      this.rootJointFrame = rootJoint.getFrameAfterJoint();
      this.twistCalculator = inverseDynamicsStructure.getTwistCalculator();

      this.shinRigidBodies = shinRigidBodies;
      this.footFrames = footFrames;

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         twistCalculator.changeTwistchangeBodyFrameNoRelativeTwist(shinRigidBodies.get(quadrant), footFrames.get(quadrant));

         String quadrantPrefix = quadrant.getCamelCaseNameForStartOfExpression();

         AlphaFilteredYoFrameVector footToRootJointPosition = AlphaFilteredYoFrameVector
               .createAlphaFilteredYoFrameVector(quadrantPrefix + "FootToRootJointPosition", "", registry, alphaFootToRootJointPosition, worldFrame);
         footToRootJointPositions.put(quadrant, footToRootJointPosition);
      }

      //XXX: to be added to the state estimator parameters (0.15 is what is used on Atlas)
      alphaRootJointLinearVelocityNewTwist.set(0.15);

      parentRegistry.addChild(registry);
   }

   public void initialize(FramePoint comPosition, QuadrantDependentList<YoFramePoint> footPositionsToPack)
   {
      updateKinematics();
      rootJointPosition.set(comPosition);
      previousRootJointPosition.set(comPosition);
      previousRootJointLinearVelocityNewTwist.setToZero();

      ArrayList<RobotQuadrant> allQuadrants = new ArrayList<>();

      for (RobotQuadrant quadrant : RobotQuadrant.values)
         allQuadrants.add(quadrant);

      estimateFeetPosition(allQuadrants, footPositionsToPack);

   }

   //for legs in contact => update CoM position
   //for legs not in contact => update foot position
   public void estimateFeetAndComPosition(ArrayList<RobotQuadrant> feetInContact, ArrayList<RobotQuadrant> feetNotInContact,
         QuadrantDependentList<YoFramePoint> footPositionsToPack, FramePoint comPositionToPack)
   {
      updateKinematics();

      estimateCoMLinearPosition(feetInContact, footPositionsToPack);

      estimateFeetPosition(feetNotInContact, footPositionsToPack);

   }

   private void updateKinematics()
   {
      rootJointPosition.setToZero();

      updateKinematicsNewTwist();
      twistCalculator.compute();

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         tempFramePoint.setToZero(rootJointFrame);
         tempFramePoint.changeFrame(footFrames.get(quadrant));

         tempFrameVector.setIncludingFrame(tempFramePoint);
         tempFrameVector.changeFrame(worldFrame);

         footToRootJointPositions.get(quadrant).update(tempFrameVector);
      }

   }

   private void updateKinematicsNewTwist()
   {
      rootJoint.packJointTwist(tempRootBodyTwist);

      rootJointLinearVelocityNewTwist.getFrameTupleIncludingFrame(tempFrameVector);
      tempFrameVector.changeFrame(tempRootBodyTwist.getExpressedInFrame());

      tempRootBodyTwist.setLinearPart(tempFrameVector);
      rootJoint.setJointTwist(tempRootBodyTwist);

      twistCalculator.compute();

      for (RobotQuadrant robotquadrant : RobotQuadrant.values)
      {
         Twist footTwistInWorld = footTwistsInWorld.get(robotquadrant);
         YoFrameVector footVelocityInWorld = footVelocitiesInWorld.get(robotquadrant);

         twistCalculator.packTwistOfBody(footTwistInWorld, shinRigidBodies.get(robotquadrant));
         footTwistInWorld.packLinearPart(tempFrameVector);

         tempFrameVector.changeFrame(worldFrame);
         footVelocityInWorld.set(tempFrameVector);
      }
   }

   private void estimateCoMLinearPosition(ArrayList<RobotQuadrant> feetInContact, QuadrantDependentList<YoFramePoint> footPositionsInWorld)
   {
      if (feetInContact.size() == 0)
      {
         rootJointPosition.set(previousRootJointPosition);
         rootJointLinearVelocityNewTwist.set(previousRootJointLinearVelocityNewTwist);
      }
      else
      {
         double scaleFactor = 1.0 / feetInContact.size();

         for (int i = 0; i < feetInContact.size(); i++)
         {
            RobotQuadrant quadrantInContact = feetInContact.get(i);

            footToRootJointPositions.get(quadrantInContact).getFrameTuple(tempPosition);
            tempPosition.scale(scaleFactor);
            rootJointPosition.add(tempPosition);
            footPositionsInWorld.get(quadrantInContact).getFrameTuple(tempPosition);
            tempPosition.scale(scaleFactor);
            rootJointPosition.add(tempPosition);

            footVelocitiesInWorld.get(quadrantInContact).getFrameTupleIncludingFrame(tempFrameVector);
            tempFrameVector.scale(scaleFactor * alphaRootJointLinearVelocityNewTwist.getDoubleValue());
            rootJointLinearVelocityNewTwist.sub(tempFrameVector);
         }
         previousRootJointPosition.set(rootJointPosition.getFrameTuple());
         previousRootJointLinearVelocityNewTwist.set(rootJointLinearVelocityNewTwist.getFrameTuple());
      }
   }

   private void estimateFeetPosition(ArrayList<RobotQuadrant> feetNotInContact, QuadrantDependentList<YoFramePoint> footPositionsInWorldToPack)
   {
      for (int i = 0; i < feetNotInContact.size(); i++)
      {
         RobotQuadrant quadrantNotInContact = feetNotInContact.get(i);
         YoFramePoint footPositionInWorld = footPositionsInWorldToPack.get(quadrantNotInContact);
         footPositionInWorld.set(footToRootJointPositions.get(quadrantNotInContact));
         footPositionInWorld.scale(-1.0);
         footPositionInWorld.add(rootJointPosition);
      }
   }

   public void getRootJointPositionAndVelocity(FramePoint positionToPack, FrameVector linearVelocityToPack)
   {
      getCoMPosition(positionToPack);
      getCoMVelocity(linearVelocityToPack);
   }

   public void getCoMPosition(FramePoint positionToPack)
   {
      rootJointPosition.getFrameTupleIncludingFrame(positionToPack);
   }

   public void getCoMVelocity(FrameVector linearVelocityToPack)
   {
      rootJointLinearVelocityNewTwist.getFrameTupleIncludingFrame(linearVelocityToPack);
   }
}
