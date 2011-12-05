package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;
import java.util.HashMap;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.kinematics.BodyPositionInTimeEstimator;
import us.ihmc.commonWalkingControlModules.kinematics.DesiredJointVelocityCalculator;
import us.ihmc.commonWalkingControlModules.kinematics.InverseKinematicsException;
import us.ihmc.commonWalkingControlModules.kinematics.LegInverseKinematicsCalculator;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointAccelerations;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Twist;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.ArtifactList;
import com.yobotics.simulationconstructionset.util.graphics.BagOfBalls;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObject;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition.GraphicType;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.splines.QuinticSplineInterpolator;

public class JointSpaceTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final QuinticSplineInterpolator hipYawSpline;
   private final QuinticSplineInterpolator[] jointSplines;

   private final YoFramePoint[] viaPoints;
   private final YoFramePoint[] viaPointsInWorldFrame;

   private final GroundTrajectoryGenerator groundTractory;
   private final int maximumNumberOfViaPoints;
   private final IntegerYoVariable numberOfViaPoints;

   private final DoubleYoVariable[] heightOfViaPoints;

   private final LegInverseKinematicsCalculator inverseKinematicsCalculator;
   private final SideDependentList<DesiredJointVelocityCalculator> desiredJointVelocityCalculators;


   private final CommonWalkingReferenceFrames referenceFrames;
   private final Orientation intermediateOrientation;
   private final SideDependentList<LegJointPositions> intermediatePositions;

   private final YoFramePoint desiredFinalLocationInWorldFrame;

   double[][] yawSplineResult = new double[1][3];
   double[][] jointSplineResult = new double[5][3];

   private final FramePoint viaPointInPelvisFrame;
   private final LegJointName[] legJointNames = new LegJointName[] { LegJointName.HIP_YAW, LegJointName.HIP_PITCH, LegJointName.HIP_ROLL, LegJointName.KNEE,
         LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL };

   // new stuff
   private final YoFramePoint initialPositionInPelvisFrame;
   private final SideDependentList<YoFramePoint> initialPositionInAnkleZUpFrame = new SideDependentList<YoFramePoint>(); 
   private final YoFramePoint finalPositionInPelvisFrame;
   private final YoFrameOrientation finalOrientationInPelvisFrame;
   private final HashMap<LegJointName, DoubleYoVariable> initialJointAngles = new HashMap<LegJointName, DoubleYoVariable>();
   private final HashMap<LegJointName, DoubleYoVariable> initialJointVelocities = new HashMap<LegJointName, DoubleYoVariable>();
   private final HashMap<LegJointName, DoubleYoVariable> initialJointAccelerations = new HashMap<LegJointName, DoubleYoVariable>();

   private final DoubleYoVariable swingDuration;

   private final EnumYoVariable<RobotSide> swingSide;

   private final DoubleYoVariable startTime;

   private final DoubleYoVariable timeToTakeForLanding;
   private final DoubleYoVariable timeToTakeForTakeOff;
   private final DoubleYoVariable averageFootVelocity;
   private final DoubleYoVariable distancePerViaPoint;
   
   private final IntegerYoVariable numberOfIKErrors;
   
   private final BodyPositionInTimeEstimator bodyPositionInTimeEstimator;
   
   private BagOfBalls bagOfBalls;
   private final YoFramePoint estimatedBodyPositionAtEndOfStep;
   private final YoFrameVector estimatedBodyVelocityAtEndOfStep;
   private final DoubleYoVariable ikAlpha;
   private final BooleanYoVariable useEstimatedJointVelocitiesAtEndOfStep;
   

   public JointSpaceTrajectoryGenerator(String name, int maximumNumberOfViaPoints, CommonWalkingReferenceFrames referenceFrames,
         LegInverseKinematicsCalculator inverseKinematicsCalculator, SideDependentList<DesiredJointVelocityCalculator> desiredJointVelocityCalculators, ProcessedSensorsInterface processedSensors, double controlDT, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
         BodyPositionInTimeEstimator bodyPositionInTimeEstimator, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(name);
      this.maximumNumberOfViaPoints = maximumNumberOfViaPoints;
      this.numberOfViaPoints = new IntegerYoVariable("numberOfViaPoints", registry);
      this.referenceFrames = referenceFrames;
      this.bodyPositionInTimeEstimator = bodyPositionInTimeEstimator;
      this.desiredJointVelocityCalculators = desiredJointVelocityCalculators;
      
      hipYawSpline = new QuinticSplineInterpolator("hipYawSpline", 2, 1, registry);
      jointSplines = new QuinticSplineInterpolator[maximumNumberOfViaPoints + 1];
      for (int i = 0; i <= maximumNumberOfViaPoints; i++)
      {
         jointSplines[i] = new QuinticSplineInterpolator("jointSpline[" + i + "]", i + 2, 5, registry);
      }

      if (maximumNumberOfViaPoints > 0)
      {
         viaPointsInWorldFrame = new YoFramePoint[maximumNumberOfViaPoints];
         groundTractory = new StraightLineGroundTrajectoryGenerator("groundTrajectory", referenceFrames, registry);
         heightOfViaPoints = new DoubleYoVariable[maximumNumberOfViaPoints];

         viaPoints = new YoFramePoint[maximumNumberOfViaPoints];

         for (int i = 0; i < maximumNumberOfViaPoints; i++)
         {
            viaPoints[i] = new YoFramePoint("viaPoint", "[" + i + "]", referenceFrames.getPelvisFrame(), registry);

            viaPointsInWorldFrame[i] = new YoFramePoint("viaPointInWorldFrame", "[" + i + "]", ReferenceFrame.getWorldFrame(), registry);

            heightOfViaPoints[i] = new DoubleYoVariable("swingHeight[" + i + "]", registry);
            heightOfViaPoints[i].set(0.03);
         }
      } else
      {
         groundTractory = null;
         viaPoints = null;
         viaPointsInWorldFrame = null;
         heightOfViaPoints = null;
      }

      desiredFinalLocationInWorldFrame = new YoFramePoint("desiredFinalSwing", "", ReferenceFrame.getWorldFrame(), registry);
      estimatedBodyPositionAtEndOfStep = new YoFramePoint("estimatedBodyPositionAtEndOfStep", "", ReferenceFrame.getWorldFrame(), registry);
      estimatedBodyVelocityAtEndOfStep = new YoFrameVector("estimatedBodyVelocityAtEndOfStep", "", ReferenceFrame.getWorldFrame(), registry);
      
      ikAlpha = new DoubleYoVariable("ikAlpha", registry);

      this.inverseKinematicsCalculator = inverseKinematicsCalculator;
      intermediateOrientation = new Orientation(referenceFrames.getPelvisFrame());
      intermediatePositions = new SideDependentList<LegJointPositions>();
      for (RobotSide side : RobotSide.values())
      {
         intermediatePositions.set(side, new LegJointPositions(side));
         initialPositionInAnkleZUpFrame.put(side, new YoFramePoint("initialPositionInOppositeAnkleZUpFrame", side.getCamelCaseNameForMiddleOfExpression(), referenceFrames.getAnkleZUpFrame(side.getOppositeSide()), registry));
      }

      viaPointInPelvisFrame = new FramePoint(referenceFrames.getPelvisFrame());

      new IntegerYoVariable("pointsRemaining", registry);

      new FramePoint(ReferenceFrame.getWorldFrame());
      createVisualizers(dynamicGraphicObjectsListRegistry, registry);

      // New Stuff
      initialPositionInPelvisFrame = new YoFramePoint("initialPositionInPelvisFrame", "", referenceFrames.getPelvisFrame(), registry);
      finalPositionInPelvisFrame = new YoFramePoint("finalPositionInPelvisFrame", "", referenceFrames.getPelvisFrame(), registry);
      finalOrientationInPelvisFrame = new YoFrameOrientation("finalOrientationInPelvisFrame", "", referenceFrames.getPelvisFrame(), registry);
      for (LegJointName jointName : legJointNames)
      {
         initialJointAngles.put(jointName, new DoubleYoVariable("q_init_" + jointName.getShortUnderBarName(), registry));
         initialJointVelocities.put(jointName, new DoubleYoVariable("qd_init_" + jointName.getShortUnderBarName(), registry));
         initialJointAccelerations.put(jointName, new DoubleYoVariable("qdd_init_" + jointName.getShortUnderBarName(), registry));
      }

      swingDuration = new DoubleYoVariable("swingDuration", registry);
      swingSide = new EnumYoVariable<RobotSide>("swingSide", registry, RobotSide.class);
      startTime = new DoubleYoVariable("startTime", registry);

      timeToTakeForLanding = new DoubleYoVariable("timeToTakeForLanding", registry);
      timeToTakeForTakeOff = new DoubleYoVariable("timeToTakeForTakeOff", registry);
      averageFootVelocity = new DoubleYoVariable("averageFootVelocity", registry);
      distancePerViaPoint = new DoubleYoVariable("distancePerViaPoint", registry);
      numberOfIKErrors = new IntegerYoVariable("numberOfIKErrors", registry);
      useEstimatedJointVelocitiesAtEndOfStep = new BooleanYoVariable("useEstimatedJointVelocitiesAtEndOfStep", registry);

      timeToTakeForLanding.set(0.1);
      timeToTakeForTakeOff.set(0.1);
      averageFootVelocity.set(1.0);
      distancePerViaPoint.set(0.15);
      useEstimatedJointVelocitiesAtEndOfStep.set(true);
      
      ikAlpha.set(0.07);

      
      parentRegistry.addChild(registry);
   }

   private void createVisualizers(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {
      if (dynamicGraphicObjectsListRegistry != null)
      {
         ArtifactList artifactList = new ArtifactList("JointSpaceTrajectory");
         ArrayList<DynamicGraphicObject> dynamicGraphicObjects = new ArrayList<DynamicGraphicObject>();

         bagOfBalls = new BagOfBalls(50, 0.02, "estimatedBodyPosition", YoAppearance.Aqua(), parentRegistry, dynamicGraphicObjectsListRegistry);

         
         DynamicGraphicPosition finalDesiredViz = desiredFinalLocationInWorldFrame.createDynamicGraphicPosition("Final Desired Swing", 0.04,
               YoAppearance.Purple(), GraphicType.BALL);
         
         DynamicGraphicVector estimatedBodyVelocity = new DynamicGraphicVector("estimated body velocity", estimatedBodyPositionAtEndOfStep, estimatedBodyVelocityAtEndOfStep, YoAppearance.Purple());

         artifactList.add(finalDesiredViz.createArtifact());
         dynamicGraphicObjects.add(finalDesiredViz);
         dynamicGraphicObjects.add(estimatedBodyVelocity);

         if (maximumNumberOfViaPoints > 0)
         {
            for (int i = 0; i < viaPointsInWorldFrame.length; i++)
            {
               DynamicGraphicPosition viaViz = viaPointsInWorldFrame[i].createDynamicGraphicPosition("Swing via point " + i, 0.03, YoAppearance.Pink(),
                     GraphicType.BALL);
               artifactList.add(viaViz.createArtifact());
               dynamicGraphicObjects.add(viaViz);
            }
         }

         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects("R2Sim02SwingSubController", dynamicGraphicObjects);

         dynamicGraphicObjectsListRegistry.registerArtifactList(artifactList);
      }
   }

   public void initialize(RobotSide swingSide, LegJointPositions currentJointPositions, LegJointVelocities currentJointVelocities,
         LegJointAccelerations currentJointAccelerations, FramePoint desiredFinalPosition, Orientation desiredOrientationIn, double swingDuration,
         int viaPointsToUse, boolean useBodyPositionEstimation)
   {
      if (viaPointsToUse > maximumNumberOfViaPoints)
         throw new RuntimeException("viaPointsToUse > maximumNumberOfViaPoints");

      this.swingSide.set(swingSide);
      this.swingDuration.set(swingDuration);
      this.numberOfViaPoints.set(viaPointsToUse);

      FramePoint initialPosition = new FramePoint(referenceFrames.getAnkleZUpFrame(swingSide));
      initialPosition.changeFrame(referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide()));
      initialPositionInAnkleZUpFrame.get(swingSide).set(initialPosition);
      initialPosition.changeFrame(referenceFrames.getPelvisFrame());
      initialPositionInPelvisFrame.set(initialPosition);

      FramePoint desiredPosition = new FramePoint(desiredFinalPosition);
      desiredPosition.changeFrame(referenceFrames.getPelvisFrame());
      Orientation desiredOrientation = new Orientation(desiredOrientationIn);
      desiredOrientation.changeFrame(referenceFrames.getPelvisFrame());

      finalPositionInPelvisFrame.set(desiredPosition);
      finalOrientationInPelvisFrame.set(desiredOrientation);
      
      for (LegJointName jointName : legJointNames)
      {
         initialJointAngles.get(jointName).set(currentJointPositions.getJointPosition(jointName));
         initialJointVelocities.get(jointName).set(currentJointVelocities.getJointVelocity(jointName));
         initialJointAccelerations.get(jointName).set(currentJointAccelerations.getJointAcceleration(jointName));
      }
      startTime.set(0.0);

      calculateSplines(0.0, useBodyPositionEstimation);

   }

   private void calculateNewEndPoint(FramePoint desiredPositionIn, Orientation desiredOrientationIn, double timeInSwing, boolean useBodyPositionEstimation)
   {
      desiredPositionIn.checkReferenceFrameMatch(referenceFrames.getPelvisFrame());

      RobotSide swingSide = this.swingSide.getEnumValue();
      LegJointPositions positions = new LegJointPositions(swingSide);
      LegJointVelocities velocities = new LegJointVelocities(legJointNames, swingSide);
      LegJointAccelerations accelerations = new LegJointAccelerations(legJointNames, swingSide);

      FramePoint currentFootPosition = new FramePoint(referenceFrames.getAnkleZUpFrame(swingSide));
      currentFootPosition.changeFrame(referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide()));
//      double height = currentFootPosition.getZ();
      currentFootPosition.changeFrame(referenceFrames.getPelvisFrame());

      compute(positions, velocities, accelerations, timeInSwing);

      if (numberOfViaPoints.getIntegerValue() == 0)
      {
         initialize(swingSide, positions, velocities, accelerations, desiredPositionIn, desiredOrientationIn, swingDuration.getDoubleValue(), 0, useBodyPositionEstimation);
         startTime.set(timeInSwing);
         return;
      }

      double timePerPoint = swingDuration.getDoubleValue() / ((double) (numberOfViaPoints.getIntegerValue() + 1));
//      int viaPointsPassed = (int) (timeInSwing / timePerPoint);
//      boolean isInFlightState = (viaPointsPassed >= 1) && (viaPointsPassed < numberOfViaPoints.getIntegerValue());
//
//      double distanceToTravel = Math.abs(currentFootPosition.distance(desiredPositionIn));
//
//      int numberOfViaPointsToUse = 0;
//      double timeRemaining = 0.6;
//      double timeRemainingInCurrentSwing = swingDuration.getDoubleValue() - (timeInSwing - startTime.getDoubleValue());
//      if (distanceToTravel < height)
//      {
//         numberOfViaPointsToUse = 0;
//         timeRemaining = (timeRemainingInCurrentSwing > timeToTakeForLanding.getDoubleValue()) ? timeRemainingInCurrentSwing : timeToTakeForLanding
//               .getDoubleValue();
//      } else if (isInFlightState)
//      {
//         numberOfViaPointsToUse = 1 + (int) Math.floor(distanceToTravel / distancePerViaPoint.getDoubleValue());
//
//         if (numberOfViaPointsToUse > maximumNumberOfViaPoints)
//            numberOfViaPointsToUse = maximumNumberOfViaPoints;
//         timeRemaining = distanceToTravel * averageFootVelocity.getDoubleValue() + timeToTakeForLanding.getDoubleValue();
//
//         if (timeRemaining < timeRemainingInCurrentSwing)
//            timeRemaining = timeRemainingInCurrentSwing;
//      } else
//      {
//         numberOfViaPointsToUse = 2 + (int) Math.floor(distanceToTravel / distancePerViaPoint.getDoubleValue());
//         if (numberOfViaPointsToUse > maximumNumberOfViaPoints)
//            numberOfViaPointsToUse = maximumNumberOfViaPoints;
//         timeRemaining = timeToTakeForTakeOff.getDoubleValue() + distanceToTravel * averageFootVelocity.getDoubleValue()
//               + timeToTakeForLanding.getDoubleValue();
//
//         if (timeRemaining < timeRemainingInCurrentSwing)
//            timeRemaining = timeRemainingInCurrentSwing;
//      }

      double timeRemaining = swingDuration.getDoubleValue() - (timeInSwing - startTime.getDoubleValue());
      int viaPointsLeft = (int)(timeRemaining/timePerPoint);
      initialize(swingSide, positions, velocities, accelerations, desiredPositionIn, desiredOrientationIn, timeRemaining, viaPointsLeft, useBodyPositionEstimation);
      startTime.set(timeInSwing);

   }

   public void updateEndPoint(FramePoint desiredPositionIn, Orientation desiredOrientationIn, double timeInSwing, boolean useBodyPositionEstimation)
   {
      FramePoint desiredPosition = new FramePoint(desiredPositionIn);
      desiredPosition.changeFrame(referenceFrames.getPelvisFrame());
      Orientation desiredOrientation = new Orientation(desiredOrientationIn);
      desiredOrientation.changeFrame(referenceFrames.getPelvisFrame());

      if (finalPositionInPelvisFrame.distance(desiredPosition) > 2e-2)
      {
         System.err.println("Calculating new endpoint, distance from previous target is " + finalPositionInPelvisFrame.distance(desiredPosition));
         calculateNewEndPoint(desiredPosition, desiredOrientation, timeInSwing, useBodyPositionEstimation);
      }

      finalPositionInPelvisFrame.set(desiredPosition);
      finalOrientationInPelvisFrame.set(desiredOrientation);

      calculateSplines(timeInSwing - startTime.getDoubleValue(), useBodyPositionEstimation);
   }

   private void calculateSplines(double timeInSwing, boolean useBodyPositionEstimation)
   {
      double[] t = new double[numberOfViaPoints.getIntegerValue() + 2];
      t[0] = 0.0;

      double[] tOfViaPoints;     
      if (numberOfViaPoints.getIntegerValue() > 0)
      {
         double timePerPoint = (swingDuration.getDoubleValue() / ((double) (numberOfViaPoints.getIntegerValue() + 1)));
         tOfViaPoints = new double[numberOfViaPoints.getIntegerValue()];
         double[] heightOfViaPointsDouble = new double[numberOfViaPoints.getIntegerValue()];
         for (int i = 1; i <= numberOfViaPoints.getIntegerValue(); i++)
         {
            tOfViaPoints[numberOfViaPoints.getIntegerValue() - i] = swingDuration.getDoubleValue() - ((double) i) * timePerPoint;
            heightOfViaPointsDouble[numberOfViaPoints.getIntegerValue() - i] = heightOfViaPoints[numberOfViaPoints.getIntegerValue() - i].getDoubleValue();
         }

         for (int i = 0; i < numberOfViaPoints.getIntegerValue(); i++)
         {
            t[i + 1] = tOfViaPoints[i];
         }

         groundTractory.getViaPoints(viaPoints, swingSide.getEnumValue(), t[0], initialPositionInAnkleZUpFrame.get(swingSide.getEnumValue()).getFramePointCopy(),
               swingDuration.getDoubleValue(), finalPositionInPelvisFrame.getFramePointCopy(), tOfViaPoints, heightOfViaPointsDouble);
      } else
      {
         tOfViaPoints = null;
      }

      t[numberOfViaPoints.getIntegerValue() + 1] = swingDuration.getDoubleValue();

      double initialYaw = initialJointAngles.get(LegJointName.HIP_YAW).getDoubleValue();
      double[] finalOrientation = finalOrientationInPelvisFrame.getYawPitchRoll(); 
      double finalYaw = finalOrientation[0];
      double finalPitch = finalOrientation[1];
      double finalRoll = finalOrientation[2];

      double yawIn[] = new double[] { initialYaw, finalYaw };
      double tYaw[] = new double[] { t[0], swingDuration.getDoubleValue() };
      hipYawSpline.initialize(tYaw);
//      hipYawSpline.determineCoefficients(0, yawIn, initialJointVelocities.get(LegJointName.HIP_YAW).getDoubleValue(), 0.0,
//            initialJointAccelerations.get(LegJointName.HIP_YAW).getDoubleValue(), 0.0);
      
      hipYawSpline.determineCoefficients(0, yawIn, 0.0, 0.0,
          0.0, 0.0);

      double[][] yIn = new double[5][numberOfViaPoints.getIntegerValue() + 2];

      for (int i = 0; i < numberOfViaPoints.getIntegerValue(); i++)
      {
         computePointOnSpline(swingSide.getEnumValue(), viaPoints[i].getFramePointCopy(), 0.0, 0.0, i + 1, tOfViaPoints[i], timeInSwing, yIn, useBodyPositionEstimation);
      }
      FrameVector upperBodyVelocityAtEndOfStep = computePointOnSpline(swingSide.getEnumValue(), finalPositionInPelvisFrame.getFramePointCopy(), finalPitch, finalRoll,
            numberOfViaPoints.getIntegerValue() + 1, swingDuration.getDoubleValue(), timeInSwing, yIn, useBodyPositionEstimation);

      QuinticSplineInterpolator currentInterpolator = jointSplines[numberOfViaPoints.getIntegerValue()];
      currentInterpolator.initialize(t);
      
      LegJointVelocities legJointVelocities = new LegJointVelocities(legJointNames, swingSide.getEnumValue());
      
      ReferenceFrame footFrame = referenceFrames.getFootFrame(swingSide.getEnumValue());
      Twist twistOfFootWithRespectToPelvis = new Twist(footFrame, ReferenceFrame.getWorldFrame(), footFrame);
      upperBodyVelocityAtEndOfStep.changeFrame(ReferenceFrame.getWorldFrame());
      twistOfFootWithRespectToPelvis.setAngularPart(upperBodyVelocityAtEndOfStep.getVector());
      

      
      desiredJointVelocityCalculators.get(swingSide.getEnumValue()).packDesiredJointVelocities(legJointVelocities, twistOfFootWithRespectToPelvis, ikAlpha.getDoubleValue());
      

      for (int j = 1; j < legJointNames.length; j++)
      {
         if(!useEstimatedJointVelocitiesAtEndOfStep.getBooleanValue())
         {
            legJointVelocities.setJointVelocity(legJointNames[j], 0.0);
         }
         yIn[j - 1][0] = initialJointAngles.get(legJointNames[j]).getDoubleValue();
         currentInterpolator.determineCoefficients(j - 1, yIn[j - 1], initialJointVelocities.get(legJointNames[j]).getDoubleValue(), legJointVelocities.getJointVelocity(legJointNames[j]),
               initialJointAccelerations.get(legJointNames[j]).getDoubleValue(), 0.0);
      }

   }

   public boolean isDoneWithSwing(double timeInSwing)
   {
      timeInSwing -= startTime.getDoubleValue();
      if (timeInSwing > swingDuration.getDoubleValue())
      {
         return true;
      }
      return false;
   }

   public double getEstimatedTimeRemaining(double timeInSwing)
   {
      if (isDoneWithSwing(timeInSwing))
         return 0.0;

      timeInSwing -= startTime.getDoubleValue();
      return swingDuration.getDoubleValue() - timeInSwing;

   }

   public void compute(LegJointPositions jointAnglesToPack, LegJointVelocities jointVelocitiesToPack, LegJointAccelerations jointAccelerationsToPack,
         double timeInSwing)
   {
      timeInSwing -= startTime.getDoubleValue();

      if (timeInSwing > swingDuration.getDoubleValue())
      {
         timeInSwing = swingDuration.getDoubleValue();
      }
      QuinticSplineInterpolator currentInterpolator = jointSplines[numberOfViaPoints.getIntegerValue()];
      currentInterpolator.compute(timeInSwing, 2, jointSplineResult);
      hipYawSpline.compute(timeInSwing, 2, yawSplineResult);

      jointAnglesToPack.setJointPosition(LegJointName.HIP_YAW, yawSplineResult[0][0]);
      jointVelocitiesToPack.setJointVelocity(LegJointName.HIP_YAW, yawSplineResult[0][1]);
      jointAccelerationsToPack.setJointAcceleration(LegJointName.HIP_YAW, yawSplineResult[0][2]);

      for (int j = 1; j < legJointNames.length; j++)
      {
         jointAnglesToPack.setJointPosition(legJointNames[j], jointSplineResult[j - 1][0]);
         jointVelocitiesToPack.setJointVelocity(legJointNames[j], jointSplineResult[j - 1][1]);
         jointAccelerationsToPack.setJointAcceleration(legJointNames[j], jointSplineResult[j - 1][2]);
      }

      updateVisualizers(swingSide.getEnumValue(), finalPositionInPelvisFrame.getFramePointCopy(), timeInSwing);
   }

   public double getSwingEndTime()
   {
      return swingDuration.getDoubleValue() + startTime.getDoubleValue();
   }

   private void updateVisualizers(RobotSide swingSide, FramePoint desiredFinalPosition, double timeInSwing)
   {
      
      if (bagOfBalls != null)
      {
         Pair<FramePose, FrameVector> poseAndVelocity = bodyPositionInTimeEstimator.getPelvisPoseAndPositionInTime(swingDuration.getDoubleValue() - timeInSwing, swingSide);
         FramePose bodyFrameInTime = poseAndVelocity.first();
         FrameVector bodyVelocityInTime = poseAndVelocity.second();
         bodyVelocityInTime.changeFrame(ReferenceFrame.getWorldFrame());
         
         FramePoint bodyPoint = bodyFrameInTime.getPositionInFrame(ReferenceFrame.getWorldFrame());
         bagOfBalls.setBallLoop(bodyPoint);
         
         estimatedBodyPositionAtEndOfStep.set(bodyPoint);
         estimatedBodyVelocityAtEndOfStep.set(bodyVelocityInTime);
         

      }

      FramePoint framePointInWorld = new FramePoint(desiredFinalPosition);
      framePointInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      desiredFinalLocationInWorldFrame.set(framePointInWorld);

      if (maximumNumberOfViaPoints > 0)
      {
         for (int i = 0; i < viaPoints.length; i++)
         {
            framePointInWorld.setAndChangeFrame(viaPoints[i].getFramePointCopy());
            framePointInWorld.changeFrame(ReferenceFrame.getWorldFrame());
            viaPointsInWorldFrame[i].set(framePointInWorld);
         }
      }
   }

   private FrameVector computePointOnSpline(RobotSide swingSide, FramePoint point, double pitch, double roll, int i, double tOfPoint, double currentTimeInSwing, double[][] yIn, boolean useBodyPositionEstimation)
   {
      double[][] yawResult = new double[1][1];
      hipYawSpline.compute(tOfPoint, 0, yawResult);
      intermediateOrientation.setYawPitchRoll(yawResult[0][0], pitch, roll);

      viaPointInPelvisFrame.setAndChangeFrame(point);
      
      Transform3D footToPelvis = computeDesiredTransform(referenceFrames.getPelvisFrame(), viaPointInPelvisFrame, intermediateOrientation);

      FrameVector ret;
      if (useBodyPositionEstimation)
      {
         Pair<FramePose, FrameVector> bodyAndVelocityInTime = bodyPositionInTimeEstimator.getPelvisPoseAndPositionInTime(tOfPoint - currentTimeInSwing, swingSide);
         FramePose pelvisPoseInTime = bodyAndVelocityInTime.first();
         Transform3D pelvisTransformInTime = new Transform3D();
         pelvisPoseInTime.getTransform3D(pelvisTransformInTime);
         pelvisTransformInTime.invert();
         footToPelvis.mul(pelvisTransformInTime, footToPelvis);

         ret = bodyAndVelocityInTime.second();
      }
      else
      {
         ret = new FrameVector(referenceFrames.getPelvisFrame());
      }
      
      try
      {
         inverseKinematicsCalculator.solve(intermediatePositions.get(swingSide), footToPelvis, swingSide, yawResult[0][0]);
      } catch (InverseKinematicsException e)
      {
         numberOfIKErrors.increment();
      }

      for (int j = 1; j < legJointNames.length; j++)
      {
         yIn[j - 1][i] = intermediatePositions.get(swingSide).getJointPosition(legJointNames[j]);
      }

      return ret;
   }

   private Transform3D computeDesiredTransform(ReferenceFrame pelvisFrame, FramePoint desiredFootPosition, Orientation desiredFootOrientation)
   {
      desiredFootOrientation.changeFrame(pelvisFrame);
      desiredFootPosition.changeFrame(pelvisFrame);
      Transform3D footToPelvis = createTransform(desiredFootOrientation, desiredFootPosition);

      return footToPelvis;
   }

   private static Transform3D createTransform(Orientation orientation, FramePoint framePoint)
   {
      orientation.checkReferenceFrameMatch(framePoint);
      Matrix3d rotationMatrix = orientation.getMatrix3d();
      Transform3D ret = new Transform3D(rotationMatrix, new Vector3d(framePoint.getPoint()), 1.0);

      return ret;
   }

}
