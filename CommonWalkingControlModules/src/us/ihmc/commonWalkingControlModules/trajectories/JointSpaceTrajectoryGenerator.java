package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.commonWalkingControlModules.kinematics.BodyPositionInTimeEstimator;
import us.ihmc.commonWalkingControlModules.kinematics.LegInverseKinematicsCalculator;
import us.ihmc.commonWalkingControlModules.kinematics.SwingLegAnglesAtEndOfStepEstimator;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointAccelerations;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.humanoidRobotics.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.graphics.BagOfBalls;
import us.ihmc.yoUtilities.graphics.YoGraphic;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.interpolators.QuinticSplineInterpolator;


public class JointSpaceTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final QuinticSplineInterpolator hipYawSpline;
   private final QuinticSplineInterpolator[] jointSplines;

   private final YoFramePoint[] viaPoints;
   private final YoFramePoint[] viaPointsInWorldFrame;

   private final GroundTrajectoryGenerator groundTrajectory;
   private final int maximumNumberOfViaPoints;
   private final IntegerYoVariable numberOfViaPoints;

   private final DoubleYoVariable[] heightOfViaPoints;

   private final CommonHumanoidReferenceFrames referenceFrames;
   private final FrameOrientation intermediateOrientation;
   private final SideDependentList<LegJointPositions> intermediatePositions;

   private final SwingLegAnglesAtEndOfStepEstimator swingLegAnglesAtEndOfStepEstimator;
   
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
   private final LinkedHashMap<LegJointName, DoubleYoVariable> initialJointAngles = new LinkedHashMap<LegJointName, DoubleYoVariable>();
   private final LinkedHashMap<LegJointName, DoubleYoVariable> initialJointVelocities = new LinkedHashMap<LegJointName, DoubleYoVariable>();
   private final LinkedHashMap<LegJointName, DoubleYoVariable> initialJointAccelerations = new LinkedHashMap<LegJointName, DoubleYoVariable>();

   private final DoubleYoVariable swingDuration;

   private final EnumYoVariable<RobotSide> swingSide;

   private final DoubleYoVariable startTime;
   
   
   
   
   private final BodyPositionInTimeEstimator bodyPositionInTimeEstimator;
   
   private BagOfBalls bagOfBalls;
   private final YoFramePoint estimatedBodyPositionAtEndOfStep;
   private final YoFrameVector estimatedBodyVelocityAtEndOfStep;
   private final DoubleYoVariable ikAlpha;
   private final BooleanYoVariable useEstimatedJointVelocitiesAtEndOfStep;
   

   public JointSpaceTrajectoryGenerator(String name, int maximumNumberOfViaPoints, CommonHumanoidReferenceFrames referenceFrames,
         LegInverseKinematicsCalculator inverseKinematicsCalculator, ProcessedSensorsInterface processedSensors, double controlDT, YoGraphicsListRegistry yoGraphicsListRegistry,
         BodyPositionInTimeEstimator bodyPositionInTimeEstimator, SwingLegAnglesAtEndOfStepEstimator swingLegAnglesAtEndOfStepEstimator, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(name);
      this.maximumNumberOfViaPoints = maximumNumberOfViaPoints;
      this.numberOfViaPoints = new IntegerYoVariable("numberOfViaPoints", registry);
      this.referenceFrames = referenceFrames;
      this.bodyPositionInTimeEstimator = bodyPositionInTimeEstimator;
      this.swingLegAnglesAtEndOfStepEstimator = swingLegAnglesAtEndOfStepEstimator;
      
      hipYawSpline = new QuinticSplineInterpolator("hipYawSpline", 2, 1, registry);
      jointSplines = new QuinticSplineInterpolator[maximumNumberOfViaPoints + 1];
      for (int i = 0; i <= maximumNumberOfViaPoints; i++)
      {
         jointSplines[i] = new QuinticSplineInterpolator("jointSpline[" + i + "]", i + 2, 5, registry);
      }

      if (maximumNumberOfViaPoints > 0)
      {
         viaPointsInWorldFrame = new YoFramePoint[maximumNumberOfViaPoints];
         groundTrajectory = new StraightLineGroundTrajectoryGenerator("groundTrajectory", referenceFrames, registry);
         heightOfViaPoints = new DoubleYoVariable[maximumNumberOfViaPoints];

         viaPoints = new YoFramePoint[maximumNumberOfViaPoints];

         for (int i = 0; i < maximumNumberOfViaPoints; i++)
         {
            viaPoints[i] = new YoFramePoint("viaPoint", "[" + i + "]", referenceFrames.getPelvisFrame(), registry);

            viaPointsInWorldFrame[i] = new YoFramePoint("viaPointInWorldFrame", "[" + i + "]", ReferenceFrame.getWorldFrame(), registry);

            heightOfViaPoints[i] = new DoubleYoVariable("swingHeight[" + i + "]", registry);
            heightOfViaPoints[i].set(0.05);
         }
      } else
      {
         groundTrajectory = null;
         viaPoints = null;
         viaPointsInWorldFrame = null;
         heightOfViaPoints = null;
      }

      desiredFinalLocationInWorldFrame = new YoFramePoint("desiredFinalSwing", "", ReferenceFrame.getWorldFrame(), registry);
      estimatedBodyPositionAtEndOfStep = new YoFramePoint("estimatedBodyPositionInWorldFrame", "", ReferenceFrame.getWorldFrame(), registry);
      estimatedBodyVelocityAtEndOfStep = new YoFrameVector("estimatedBodyVelocityInWorldFrame", "", ReferenceFrame.getWorldFrame(), registry);
      
      ikAlpha = new DoubleYoVariable("ikAlpha", registry);

      intermediateOrientation = new FrameOrientation(referenceFrames.getPelvisFrame());
      intermediatePositions = new SideDependentList<LegJointPositions>();
      for (RobotSide side : RobotSide.values)
      {
         intermediatePositions.set(side, new LegJointPositions(side));
         initialPositionInAnkleZUpFrame.put(side, new YoFramePoint("initialPositionInOppositeAnkleZUpFrame", side.getCamelCaseNameForMiddleOfExpression(), referenceFrames.getAnkleZUpFrame(side.getOppositeSide()), registry));
      }

      viaPointInPelvisFrame = new FramePoint(referenceFrames.getPelvisFrame());

      new IntegerYoVariable("pointsRemaining", registry);

      new FramePoint(ReferenceFrame.getWorldFrame());
      createVisualizers(yoGraphicsListRegistry, registry);

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

      useEstimatedJointVelocitiesAtEndOfStep = new BooleanYoVariable("useEstimatedJointVelocitiesAtEndOfStep", registry);

      useEstimatedJointVelocitiesAtEndOfStep.set(true);
      
      ikAlpha.set(0.07);

      

      
      parentRegistry.addChild(registry);
   }

   private void createVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      if (yoGraphicsListRegistry != null)
      {
         ArtifactList artifactList = new ArtifactList("JointSpaceTrajectory");
         ArrayList<YoGraphic> yoGraphics = new ArrayList<YoGraphic>();

         bagOfBalls = new BagOfBalls(50, 0.02, "estimatedBodyPosition", YoAppearance.Aqua(), parentRegistry, yoGraphicsListRegistry);

         
         YoGraphicPosition finalDesiredViz = new YoGraphicPosition("Final Desired Swing", desiredFinalLocationInWorldFrame, 0.04,
               YoAppearance.Purple(), GraphicType.BALL);
         
         YoGraphicVector estimatedBodyVelocity = new YoGraphicVector("estimated body velocity", estimatedBodyPositionAtEndOfStep, estimatedBodyVelocityAtEndOfStep, YoAppearance.Purple());

         artifactList.add(finalDesiredViz.createArtifact());
         yoGraphics.add(finalDesiredViz);
         yoGraphics.add(estimatedBodyVelocity);

         if (maximumNumberOfViaPoints > 0)
         {
            for (int i = 0; i < viaPointsInWorldFrame.length; i++)
            {
               YoGraphicPosition viaViz = new YoGraphicPosition("Swing via point " + i, viaPointsInWorldFrame[i], 0.03, YoAppearance.Pink(),
                     GraphicType.BALL);
               artifactList.add(viaViz.createArtifact());
               yoGraphics.add(viaViz);
            }
         }

         yoGraphicsListRegistry.registerYoGraphics("R2Sim02SwingSubController", yoGraphics);

         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }
   }

   public void initialize(RobotSide swingSide, LegJointPositions currentJointPositions, LegJointVelocities currentJointVelocities,
         LegJointAccelerations currentJointAccelerations, FramePoint desiredFinalPosition, FrameOrientation desiredOrientationIn, double swingDuration,
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
      FrameOrientation desiredOrientation = new FrameOrientation(desiredOrientationIn);
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

   private void calculateNewEndPoint(FramePoint desiredPositionIn, FrameOrientation desiredOrientationIn, double timeInSwing, boolean useBodyPositionEstimation)
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

   public void updateEndPoint(FramePoint desiredPositionIn, FrameOrientation desiredOrientationIn, double timeInSwing, boolean useBodyPositionEstimation)
   {
      FramePoint desiredPosition = new FramePoint(desiredPositionIn);
      desiredPosition.changeFrame(referenceFrames.getPelvisFrame());
      FrameOrientation desiredOrientation = new FrameOrientation(desiredOrientationIn);
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

         groundTrajectory.getViaPoints(viaPoints, swingSide.getEnumValue(), t[0], initialPositionInAnkleZUpFrame.get(swingSide.getEnumValue()).getFramePointCopy(),
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
      LegJointVelocities legJointVelocities = new LegJointVelocities(legJointNames, swingSide.getEnumValue());

      for (int i = 0; i < numberOfViaPoints.getIntegerValue(); i++)
      {
         computePointOnSpline(swingSide.getEnumValue(), viaPoints[i].getFramePointCopy(), 0.0, 0.0, i + 1, tOfViaPoints[i], timeInSwing, yIn, null, useBodyPositionEstimation);
      }
      computePointOnSpline(swingSide.getEnumValue(), finalPositionInPelvisFrame.getFramePointCopy(), finalPitch, finalRoll,
            numberOfViaPoints.getIntegerValue() + 1, swingDuration.getDoubleValue(), timeInSwing, yIn, legJointVelocities, useBodyPositionEstimation);

      QuinticSplineInterpolator currentInterpolator = jointSplines[numberOfViaPoints.getIntegerValue()];
      currentInterpolator.initialize(t);
      
      
      for (int j = 1; j < legJointNames.length; j++)
      {
         if(!useEstimatedJointVelocitiesAtEndOfStep.getBooleanValue() || !useBodyPositionEstimation)
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
         ImmutablePair<FramePose, FrameVector> poseAndVelocity = bodyPositionInTimeEstimator.getPelvisPoseAndVelocityInTime(swingDuration.getDoubleValue() - timeInSwing, swingSide);
         FramePose bodyFrameInTime = poseAndVelocity.getLeft();
         FrameVector bodyVelocityInTime = poseAndVelocity.getRight();
         bodyVelocityInTime.changeFrame(ReferenceFrame.getWorldFrame());
         
         FramePoint bodyPoint = new FramePoint();
         bodyFrameInTime.getPositionIncludingFrame(bodyPoint);
         bodyPoint.changeFrame(ReferenceFrame.getWorldFrame());
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
            framePointInWorld.setIncludingFrame(viaPoints[i].getFramePointCopy());
            framePointInWorld.changeFrame(ReferenceFrame.getWorldFrame());
            viaPointsInWorldFrame[i].set(framePointInWorld);
         }
      }
   }

   private void computePointOnSpline(RobotSide swingSide, FramePoint point, double pitch, double roll, int i, double tOfPoint, double currentTimeInSwing, double[][] yIn, LegJointVelocities legJointVelocities, boolean useBodyPositionEstimation)
   {
      double[][] yawResult = new double[1][1];
      hipYawSpline.compute(tOfPoint, 0, yawResult);
      intermediateOrientation.setYawPitchRoll(yawResult[0][0], pitch, roll);

      viaPointInPelvisFrame.setIncludingFrame(point);
      
      
      double timeRemaining = tOfPoint - currentTimeInSwing;
      
      
      
      
      swingLegAnglesAtEndOfStepEstimator.getEstimatedJointAnglesAtEndOfStep(intermediatePositions.get(swingSide), legJointVelocities, swingSide,
            viaPointInPelvisFrame, intermediateOrientation, yawResult[0][0], timeRemaining, useBodyPositionEstimation);

      for (int j = 1; j < legJointNames.length; j++)
      {
         yIn[j - 1][i] = intermediatePositions.get(swingSide).getJointPosition(legJointNames[j]);
      }

   }

   

}
