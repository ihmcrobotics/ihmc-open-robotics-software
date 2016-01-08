package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import java.util.ArrayList;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
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
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class CenterOfMassKinematicBasedCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private SixDoFJoint rootJoint;
   private SixDoFJointReferenceFrame rootJointFrame;
   private TwistCalculator twistCalculator;

   private final YoFramePoint rootJointPosition = new YoFramePoint("estimatedRootJointPositionWithKinematics", worldFrame, registry);
   private final YoFrameVector rootJointLinearVelocityNewTwist = new YoFrameVector("estimatedRootJointVelocityNewTwist", worldFrame, registry);
   private final YoFrameQuaternion rootJointYoOrientation = new YoFrameQuaternion("estimatedRootJointOrientationWithKinematics", worldFrame, registry);
   private final FrameOrientation rootJointOrientation = new FrameOrientation(worldFrame);

   
   private final YoFramePoint previousRootJointPosition = new YoFramePoint("previousEstimatedRootJointPositionWithKinematics", worldFrame, registry);
   private final YoFrameVector previousRootJointLinearVelocityNewTwist = new YoFrameVector("previousEstimatedRootJointVelocityNewTwist", worldFrame, registry);
   private final YoFrameQuaternion previousRootJointOrientation = new YoFrameQuaternion("previousEstimatedRootJointOrientationWithKinematics", worldFrame,
         registry);

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

   private final QuadrantDependentList<FramePoint> estimatedFeetPositions = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFramePoint> yoEstimatedFeetPositions;
   private final QuadrantDependentList<ThreePointsAverageReferenceFrame> estimatedFeetAverageReferenceFrames = new QuadrantDependentList<>();

   private final QuadrantDependentList<FramePoint> calculatedFeetPositions = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFramePoint> yoCalculatedFeetPositions = new QuadrantDependentList<>();
   private final QuadrantDependentList<ThreePointsAverageReferenceFrame> calculatedFeetAverageReferenceFrames = new QuadrantDependentList<>();

   private final EnumYoVariable<RobotQuadrant> yoUsedQuadrant = new EnumYoVariable<RobotQuadrant>("usedQuadrantForOrientationEstimation", registry,
         RobotQuadrant.class, true);

   private final ArrayList<RobotQuadrant> allQuadrants = new ArrayList<>();

   private final RootJointOrientationCorrectorHelper rootJointOrientationCorrectorHelper = new RootJointOrientationCorrectorHelper("stateEstimator", registry); 
   
   private final FrameOrientation orientationError = new FrameOrientation();
   
    //Visualization Variables
   private final ArrayList<YoGraphicReferenceFrame> estimatedFeetAverageReferenceFrameVizs = new ArrayList<>();
   private final ArrayList<YoGraphicReferenceFrame> calculatedFeetAverageReferenceFrameVizs = new ArrayList<>();
   
   //Temporary Variables
   private final Twist tempRootBodyTwist = new Twist();
   private final FrameVector tempFrameVector = new FrameVector();
   private final FramePoint tempFramePoint = new FramePoint();
   private final FramePoint tempPosition = new FramePoint();
   private final FrameOrientation tempOrientation = new FrameOrientation();
   private final Quat4d tempQuaternion = new Quat4d();
   private final AxisAngle4d tempAxisAngle = new AxisAngle4d();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public CenterOfMassKinematicBasedCalculator(FullInverseDynamicsStructure inverseDynamicsStructure, QuadrantDependentList<RigidBody> shinRigidBodies,
         QuadrantDependentList<ReferenceFrame> footFrames, QuadrantDependentList<YoFramePoint> yoEstimatedFootPositions, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry graphicsListRegistry)
   {
      this.rootJoint = inverseDynamicsStructure.getRootJoint();
      this.rootJointFrame = rootJoint.getFrameAfterJoint();
      this.twistCalculator = inverseDynamicsStructure.getTwistCalculator();

      this.shinRigidBodies = shinRigidBodies;
      this.footFrames = footFrames;

      this.yoEstimatedFeetPositions = yoEstimatedFootPositions;

      for (RobotQuadrant quadrant : RobotQuadrant.values)
         allQuadrants.add(quadrant);

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         twistCalculator.changeTwistchangeBodyFrameNoRelativeTwist(shinRigidBodies.get(quadrant), footFrames.get(quadrant));

         String quadrantPrefix = quadrant.getCamelCaseNameForStartOfExpression();

         AlphaFilteredYoFrameVector footToRootJointPosition = AlphaFilteredYoFrameVector
               .createAlphaFilteredYoFrameVector(quadrantPrefix + "FootToRootJointPosition", "", registry, alphaFootToRootJointPosition, worldFrame);
         footToRootJointPositions.put(quadrant, footToRootJointPosition);

         FramePoint estimatedFootPosition = new FramePoint(worldFrame);
         estimatedFeetPositions.put(quadrant, estimatedFootPosition);

         FramePoint calculatedFootPosition = new FramePoint(worldFrame);
         calculatedFeetPositions.put(quadrant, calculatedFootPosition);

         YoFramePoint yoCalculatedFootPosition = new YoFramePoint(quadrantPrefix + "CalculatedFootPosition", worldFrame, registry);
         yoCalculatedFeetPositions.put(quadrant, yoCalculatedFootPosition);

         YoGraphicPosition calculatedFootPositionViz = new YoGraphicPosition(quadrant.getCamelCaseNameForStartOfExpression() + "CalculatedPosition",
               yoCalculatedFootPosition, 0.02, YoAppearance.Green());
         graphicsListRegistry.registerYoGraphic("KinematicsBasedStateEstimatorCalculated", calculatedFootPositionViz);

      }

      //this one needs to be in a difference for loop because all the feetPosition FramePoints have to be previously created
      constructAverageFrame(RobotQuadrant.FRONT_LEFT, RobotQuadrant.FRONT_RIGHT, RobotQuadrant.HIND_RIGHT, RobotQuadrant.HIND_LEFT, graphicsListRegistry);
      constructAverageFrame(RobotQuadrant.FRONT_RIGHT, RobotQuadrant.HIND_RIGHT, RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_LEFT, graphicsListRegistry);
      constructAverageFrame(RobotQuadrant.HIND_RIGHT, RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_LEFT, RobotQuadrant.FRONT_RIGHT, graphicsListRegistry);
      constructAverageFrame(RobotQuadrant.HIND_LEFT, RobotQuadrant.FRONT_LEFT, RobotQuadrant.FRONT_RIGHT, RobotQuadrant.HIND_RIGHT, graphicsListRegistry);

      //XXX: to be added to the state estimator parameters (0.15 is what is used on Atlas)
      alphaRootJointLinearVelocityNewTwist.set(0.15);

      parentRegistry.addChild(registry);
   }

   private void constructAverageFrame(RobotQuadrant quadrant1, RobotQuadrant quadrant2, RobotQuadrant quadrant3, RobotQuadrant quadrantNotUsed,
         YoGraphicsListRegistry graphicsListRegistry)
   {
      String prefix = quadrantNotUsed.getCamelCaseNameForStartOfExpression();

      //construction of the estimated related variables
      FramePoint estimatedP1 = estimatedFeetPositions.get(quadrant1);
      FramePoint estimatedP2 = estimatedFeetPositions.get(quadrant2);
      FramePoint estimatedP3 = estimatedFeetPositions.get(quadrant3);

      ThreePointsAverageReferenceFrame estimatedFootAverageReferenceFrame = new ThreePointsAverageReferenceFrame(prefix + "EstimatedFootAverageReferenceFrame",
            estimatedP1, estimatedP2, estimatedP3, worldFrame);
      estimatedFeetAverageReferenceFrames.put(quadrantNotUsed, estimatedFootAverageReferenceFrame);

      YoGraphicReferenceFrame estimatedFootAverageRefViz = new YoGraphicReferenceFrame(estimatedFootAverageReferenceFrame, registry, 0.15,
            YoAppearance.Yellow());
      graphicsListRegistry.registerYoGraphic(prefix + "KinematicsBasedStateEstimatorEstimated", estimatedFootAverageRefViz);
      estimatedFeetAverageReferenceFrameVizs.add(estimatedFootAverageRefViz);

      //construction of the calculated related variables
      FramePoint calculatedP1 = calculatedFeetPositions.get(quadrant1);
      FramePoint calculatedP2 = calculatedFeetPositions.get(quadrant2);
      FramePoint calculatedP3 = calculatedFeetPositions.get(quadrant3);

      ThreePointsAverageReferenceFrame calculatedFootAverageReferenceFrame = new ThreePointsAverageReferenceFrame(
            prefix + "CalculatedFootAverageReferenceFrame", calculatedP1, calculatedP2, calculatedP3, worldFrame);
      calculatedFeetAverageReferenceFrames.put(quadrantNotUsed, calculatedFootAverageReferenceFrame);

      YoGraphicReferenceFrame calculatedFootAverageRefViz = new YoGraphicReferenceFrame(calculatedFootAverageReferenceFrame, registry, 0.2,
            YoAppearance.Green());
      graphicsListRegistry.registerYoGraphic(prefix + "KinematicsBasedStateEstimatorCalculated", calculatedFootAverageRefViz);
      calculatedFeetAverageReferenceFrameVizs.add(calculatedFootAverageRefViz);
   }

   public void initialize(FramePoint comInitialPosition, FrameOrientation comInitialOrientation)//, QuadrantDependentList<YoFramePoint> footPositionsToPack)
   {
      rootJointOrientationCorrectorHelper.setCorrectionAlpha(0.1);
      
      rootJointPosition.set(comInitialPosition);
      previousRootJointPosition.set(comInitialPosition);
      previousRootJointLinearVelocityNewTwist.setToZero();

      rootJointYoOrientation.set(comInitialOrientation);
      previousRootJointOrientation.set(rootJointYoOrientation);

      yoUsedQuadrant.set(usedQuadrant);

      updateKinematics();

      estimateFeetPosition(rootJointPosition, rootJointYoOrientation, allQuadrants, yoEstimatedFeetPositions);

      for (RobotQuadrant quadrant : RobotQuadrant.values)
         yoCalculatedFeetPositions.get(quadrant).set(yoEstimatedFeetPositions.get(quadrant));

      updateAverageFrames();

      updateViz();

   }

   private void updateAverageFrames()
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         yoEstimatedFeetPositions.get(quadrant).getFrameTuple(estimatedFeetPositions.get(quadrant));
         yoCalculatedFeetPositions.get(quadrant).getFrameTuple(calculatedFeetPositions.get(quadrant));
      }

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         estimatedFeetAverageReferenceFrames.get(quadrant).update();
         calculatedFeetAverageReferenceFrames.get(quadrant).update();
      }
   }

   private RobotQuadrant usedQuadrant = RobotQuadrant.HIND_LEFT;

   //for legs in contact => update CoM position
   //for legs not in contact => update foot position
   public void estimateFeetAndComPositionAndOrientation(ArrayList<RobotQuadrant> feetInContact, ArrayList<RobotQuadrant> feetNotInContact)
   {
      updateKinematics();
      
      //estimate Orientation 
      estimateCoMOrientation(feetNotInContact);

      //estimate linear position
      estimateCoMLinearPosition(feetInContact);

      //estimate the position of the swing foot
      estimateFeetPosition(rootJointPosition, rootJointYoOrientation, feetNotInContact, yoEstimatedFeetPositions);

      updateViz();
   }

   private void updateKinematics()
   {
      rootJointPosition.setToZero();
      rootJointYoOrientation.set(0.0, 0.0, 0.0);

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

   private void estimateCoMOrientation(ArrayList<RobotQuadrant> feetNotInContact)
   {
//      //estimate feet position with the previous estimate and the current joint angles, and store them in an array of calculated feet position
//      estimateFeetPosition(previousRootJointPosition, previousRootJointOrientation, allQuadrants, yoCalculatedFeetPositions);
//      updateAverageFrames();

      //compare orientation
      if (feetNotInContact.size() == 0)
      {
         //estimate feet position with the previous estimate and the current joint angles, and store them in an array of calculated feet position
         estimateFeetPosition(previousRootJointPosition, previousRootJointOrientation, allQuadrants, yoCalculatedFeetPositions);
         updateAverageFrames();
         calculateOrientationError(usedQuadrant);
      }
      else if (feetNotInContact.size() == 1)
      {
         //estimate feet position with the previous estimate and the current joint angles, and store them in an array of calculated feet position
         estimateFeetPosition(previousRootJointPosition, previousRootJointOrientation, allQuadrants, yoCalculatedFeetPositions);
         updateAverageFrames();
         //         usedQuadrant = feetNotInContact.get(0); XXX uncomment that line to make things work
         yoUsedQuadrant.set(usedQuadrant);
         calculateOrientationError(usedQuadrant);
      }
      else
      {
         rootJointYoOrientation.set(previousRootJointOrientation);
      }
   }

   private void calculateOrientationError(RobotQuadrant quadrant)
   {
      orientationError.setToZero(estimatedFeetAverageReferenceFrames.get(quadrant));
      orientationError.changeFrame(calculatedFeetAverageReferenceFrames.get(quadrant));
      rootJointOrientationCorrectorHelper.compensateOrientationError(rootJointOrientation, orientationError);
      rootJointYoOrientation.set(rootJointOrientation);
      previousRootJointOrientation.set(rootJointOrientation);
   }

   private void estimateCoMLinearPosition(ArrayList<RobotQuadrant> feetInContact)//, QuadrantDependentList<YoFramePoint> footPositionsInWorld)
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
            yoEstimatedFeetPositions.get(quadrantInContact).getFrameTuple(tempPosition);
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

   private void estimateFeetPosition(YoFramePoint rootPosition, YoFrameQuaternion rootOrientation, ArrayList<RobotQuadrant> feetNotInContact,
         QuadrantDependentList<YoFramePoint> footPositionsInWorldToPack)
   {
      for (int i = 0; i < feetNotInContact.size(); i++)
      {
         RobotQuadrant quadrantNotInContact = feetNotInContact.get(i);
         YoFramePoint footPositionInWorld = footPositionsInWorldToPack.get(quadrantNotInContact);
         footPositionInWorld.set(footToRootJointPositions.get(quadrantNotInContact));
         footPositionInWorld.scale(-1.0);
         footPositionInWorld.add(rootPosition);

         rootOrientation.get(tempQuaternion);
         tempTransform.setRotationAndZeroTranslation(tempQuaternion);
         footPositionInWorld.applyTransform(tempTransform);
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

   public void getCoMOrientation(FrameOrientation orientationToPack)
   {
      rootJointYoOrientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   private void updateViz()
   {
      for (int i = 0; i < estimatedFeetAverageReferenceFrameVizs.size(); i++)
      {
         estimatedFeetAverageReferenceFrameVizs.get(i).update();
         calculatedFeetAverageReferenceFrameVizs.get(i).update();
      }
   }

}
