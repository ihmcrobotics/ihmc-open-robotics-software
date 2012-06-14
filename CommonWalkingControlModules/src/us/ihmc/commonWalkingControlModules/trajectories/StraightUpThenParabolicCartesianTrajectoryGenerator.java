package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.trajectory.YoMinimumJerkTrajectory;
import com.yobotics.simulationconstructionset.util.trajectory.YoParabolicTrajectoryGenerator;

public class StraightUpThenParabolicCartesianTrajectoryGenerator implements CartesianTrajectoryGenerator
{
   private final String namePostFix = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final YoMinimumJerkTrajectory straightUpParameterTrajectory;
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

      this.straightUpParameterTrajectory = new YoMinimumJerkTrajectory(namePrefix + "StraightUp", registry);
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

   public void initialize(FramePoint initialPosition, FrameVector initialVelocity, FramePoint finalDesiredPosition)
   {
      initialPosition.checkReferenceFrameMatch(finalDesiredPosition);
      this.initialPosition.set(initialPosition);
      FramePoint straightUpPosition = new FramePoint(initialPosition);
      straightUpPosition.setZ(Math.max(finalDesiredPosition.getZ(), initialPosition.getZ()));
      straightUpVector.set(straightUpPosition);
      straightUpVector.sub(initialPosition);
      
      straightUpTime.set(straightUpVector.getZ() / straightUpAverageVelocity.getDoubleValue());
      
      double dPositiondParabolicParameterInitial = computeDPositionDParabolicParameterInitial(straightUpPosition, finalDesiredPosition);

      timeIntoStep.set(0.0);

      double dStraightUpParameterdTime = 1.0;
      double dPositiondStraightUpParameter = straightUpVector.getZ();
      straightUpParameterTrajectory.setParams(0.0, 0.0, 0.0, 1.0, dStraightUpParameterdTime, 0.0, 0.0, straightUpTime.getDoubleValue());

      double dParabolicParameterdTime = dPositiondStraightUpParameter * dStraightUpParameterdTime / dPositiondParabolicParameterInitial;
      parabolicParameterTrajectory.setParams(0.0, dParabolicParameterdTime, 0.0, 1.0, 0.0, 0.0, straightUpTime.getDoubleValue(), straightUpTime.getDoubleValue() + parabolicTime.getDoubleValue());

      FrameVector parabolaInitialVelocity = new FrameVector(initialPosition.getReferenceFrame(), 0.0, 0.0, dPositiondParabolicParameterInitial);
      parabolicTrajectoryGenerator.initialize(straightUpPosition, parabolaInitialVelocity, finalDesiredPosition);
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

      if (timeIntoStep.getDoubleValue() < straightUpParameterTrajectory.getFinalTime())
      {
         straightUpParameterTrajectory.computeTrajectory(timeIntoStep.getDoubleValue());
         double parameter = straightUpParameterTrajectory.getPosition();
         double parameterd = straightUpParameterTrajectory.getVelocity();
         double parameterdd = straightUpParameterTrajectory.getAcceleration();
         parameter = MathTools.clipToMinMax(parameter, 0.0, 1.0);

         straightUpVector.getFrameVectorAndChangeFrameOfPackedVector(tempVector);
         tempVector.scale(parameter);
         initialPosition.getFramePoint(positionToPack);
         positionToPack.add(tempVector);

         straightUpVector.getFrameVector(velocityToPack);
         velocityToPack.scale(parameterd);

         straightUpVector.getFrameVector(accelerationToPack);
         accelerationToPack.scale(parameterdd);
      }
      else
      {
         parabolicParameterTrajectory.computeTrajectory(timeIntoStep.getDoubleValue());
         double parameter = parabolicParameterTrajectory.getPosition();
         double parameterd = parabolicParameterTrajectory.getVelocity();
         double parameterdd = parabolicParameterTrajectory.getAcceleration();
         parameter = MathTools.clipToMinMax(parameter, 0.0, 1.0);

         parabolicTrajectoryGenerator.packPosition(positionToPack, parameter);

         parabolicTrajectoryGenerator.packVelocity(tempVector, parameter);
         velocityToPack.set(tempVector);
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
