package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.CartesianTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoMinimumJerkTrajectory;
import us.ihmc.robotics.math.trajectories.YoParabolicTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public class StraightUpThenParabolicCartesianTrajectoryGenerator implements CartesianTrajectoryGenerator
{
   private final String namePostFix = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final YoPolynomial straightUpParameterTrajectory;
   private final YoMinimumJerkTrajectory parabolicParameterTrajectory;
   private final YoParabolicTrajectoryGenerator parabolicTrajectoryGenerator;
   private final DoubleYoVariable groundClearance;
   private final DoubleYoVariable straightUpAverageVelocity;
   private final DoubleYoVariable straightUpTime;
   private final DoubleYoVariable parabolicTime;
   private final DoubleYoVariable timeIntoStep;

   private final YoFramePoint initialPosition;
   private final YoFrameVector straightUpVector;

   public StraightUpThenParabolicCartesianTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, double straightUpAverageVelocity, double parabolicTime,
           double groundClearance, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + namePostFix);

      this.straightUpParameterTrajectory = new YoPolynomial(namePrefix + "StraightUp", 6, registry);
      this.parabolicParameterTrajectory = new YoMinimumJerkTrajectory(namePrefix + "Parabolic", registry);
      this.parabolicTrajectoryGenerator = new YoParabolicTrajectoryGenerator(namePrefix, referenceFrame, registry);
      this.groundClearance = new DoubleYoVariable("groundClearance", registry);
      this.straightUpAverageVelocity = new DoubleYoVariable("straightUpAverageVelocity", registry);
      this.straightUpTime = new DoubleYoVariable("straightUpTime", registry);
      this.parabolicTime = new DoubleYoVariable("stepTime", registry);
      this.timeIntoStep = new DoubleYoVariable("timeIntoStep", registry);
      this.initialPosition = new YoFramePoint("initialPosition", "", referenceFrame, registry);
      this.straightUpVector = new YoFrameVector("straightUpVector", "", referenceFrame, registry);
      parentRegistry.addChild(registry);

      MathTools.checkIfInRange(straightUpAverageVelocity, 0.0, Double.POSITIVE_INFINITY);
      MathTools.checkIfInRange(parabolicTime, 0.0, Double.POSITIVE_INFINITY);

      this.straightUpAverageVelocity.set(straightUpAverageVelocity);
      this.parabolicTime.set(parabolicTime);
      this.groundClearance.set(groundClearance);
   }

   public void initialize(FramePoint initialPosition, FrameVector initialVelocity, FrameVector initialAcceleration, FramePoint finalDesiredPosition, FrameVector finalDesiredVelocity)
   {
      initialPosition.checkReferenceFrameMatch(finalDesiredPosition);
      this.initialPosition.set(initialPosition);
      FramePoint straightUpPosition = new FramePoint(initialPosition);
      straightUpPosition.setZ(Math.max(finalDesiredPosition.getZ(), initialPosition.getZ()));
      straightUpVector.set(straightUpPosition);
      straightUpVector.sub(initialPosition);
      
      straightUpTime.set(straightUpVector.getZ() / straightUpAverageVelocity.getDoubleValue());
      
      double dPositiondParabolicParameter = computeDPositionDParabolicParameterInitial(straightUpPosition, finalDesiredPosition);
      FrameVector parabolaInitialVelocity = new FrameVector(initialPosition.getReferenceFrame(), 0.0, 0.0, dPositiondParabolicParameter);
      parabolicTrajectoryGenerator.initialize(straightUpPosition, parabolaInitialVelocity, finalDesiredPosition);

      timeIntoStep.set(0.0);

//      straightUpParameterTrajectory.setCubic(0, straightUpTime.getDoubleValue(), 0.0, 0.0, 1.0, straightUpParameterDot);
//      straightUpParameterTrajectory.setQuadratic(0, straightUpTime.getDoubleValue(), 0.0, 1.0, 1.0);
      double zdf = 3.0; // TODO: do something smart here
      straightUpParameterTrajectory.setQuintic(0.0, straightUpTime.getDoubleValue(), 0.0, 0.0, 0.0, 1.0, zdf, 0.0);
      straightUpParameterTrajectory.compute(straightUpTime.getDoubleValue());
      double straightUpParameterDot = straightUpParameterTrajectory.getVelocity();
      double straightUpParameterDDot = straightUpParameterTrajectory.getAcceleration();

      double dPositiondStraightUpParameter = straightUpVector.getZ();
      FrameVector parabolicAcceleration = new FrameVector(initialPosition.getReferenceFrame());
      parabolicTrajectoryGenerator.packAcceleration(parabolicAcceleration);
      double parabolicParameterDot = dPositiondStraightUpParameter / dPositiondParabolicParameter * straightUpParameterDot;
      double parabolicParameterDDot = (dPositiondStraightUpParameter * straightUpParameterDDot - parabolicAcceleration.getZ() * MathTools.square(parabolicParameterDot)) / dPositiondParabolicParameter;
      parabolicParameterTrajectory.setParams(0.0, parabolicParameterDot, parabolicParameterDDot, 1.0, 1.0, 0.0, straightUpTime.getDoubleValue(), straightUpTime.getDoubleValue() + parabolicTime.getDoubleValue());


   }

   private double computeDPositionDParabolicParameterInitial(FramePoint initialPosition, FramePoint finalDesiredPosition)
   {
      double z0 = initialPosition.getZ();
      double z1 = finalDesiredPosition.getZ();
      double zMax = z1 + groundClearance.getDoubleValue();

      return 2.0 * (Math.sqrt((z0 - zMax) * (z1 - zMax)) + zMax - z0);
   }

   private final FrameVector tempVector = new FrameVector(ReferenceFrame.getWorldFrame());

   public void computeNextTick(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack, double deltaT)
   {
      timeIntoStep.add(deltaT);
      double totalTime = parabolicParameterTrajectory.getFinalTime();
      if (timeIntoStep.getDoubleValue() > totalTime)
         timeIntoStep.set(totalTime);

      if (timeIntoStep.getDoubleValue() < straightUpTime.getDoubleValue())
      {
         straightUpParameterTrajectory.compute(timeIntoStep.getDoubleValue());
         double parameter = straightUpParameterTrajectory.getPosition();
         double parameterd = straightUpParameterTrajectory.getVelocity();
         double parameterdd = straightUpParameterTrajectory.getAcceleration();
         parameter = MathTools.clipToMinMax(parameter, 0.0, 1.0);

         straightUpVector.getFrameTupleIncludingFrame(tempVector);
         tempVector.scale(parameter);
         initialPosition.getFrameTupleIncludingFrame(positionToPack);
         positionToPack.add(tempVector);

         straightUpVector.getFrameTupleIncludingFrame(velocityToPack);
         velocityToPack.scale(parameterd);

         straightUpVector.getFrameTupleIncludingFrame(accelerationToPack);
         accelerationToPack.scale(parameterdd);
      }
      else
      {
         parabolicParameterTrajectory.computeTrajectory(timeIntoStep.getDoubleValue());
         double parameter = parabolicParameterTrajectory.getPosition();
         double parameterd = parabolicParameterTrajectory.getVelocity();
         double parameterdd = parabolicParameterTrajectory.getAcceleration();
         
         if (parameter < 0.0)
         {
            parameter = 0.0;
            parameterd = 0.0;
            parameterdd = 0.0;
         }
         
         if (parameter > 1.0)
         {
            parameter = 1.0;
            parameterd = 0.0;
            parameterdd = 0.0;
         }

         parabolicTrajectoryGenerator.packPosition(positionToPack, parameter);

         parabolicTrajectoryGenerator.packVelocity(tempVector, parameter);
         velocityToPack.setIncludingFrame(tempVector);
         velocityToPack.scale(parameterd);

         parabolicTrajectoryGenerator.packAcceleration(accelerationToPack);
         accelerationToPack.scale(parameterd * parameterd);
         tempVector.scale(parameterdd);
         accelerationToPack.add(tempVector);
      }
   }

   public void updateFinalDesiredPosition(FramePoint finalDesiredPosition)
   {
      // empty
   }

   public ReferenceFrame getReferenceFrame()
   {
      return parabolicTrajectoryGenerator.getReferenceFrame();
   }

   public boolean isDone()
   {
      return timeIntoStep.getDoubleValue() >= parabolicParameterTrajectory.getFinalTime();
   }

   public double getFinalTime()
   {
      return parabolicParameterTrajectory.getFinalTime();
   }
}
