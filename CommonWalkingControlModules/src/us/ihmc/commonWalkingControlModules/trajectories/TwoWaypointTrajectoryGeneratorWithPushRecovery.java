package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.BagOfBalls;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryParametersProvider;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryWaypointGenerationMethod;
import com.yobotics.simulationconstructionset.util.trajectory.VectorProvider;

/** 
 * 
 * @author anonymous
 *
 * This trajectory unless the robot is pushed, this class will behave exactly like the TwoWaypointTrajectoyrGenerator except this class has 
 * the soft TouchdownTrajectoryGenerator included rather than the two being combined in an ArrayList of position trajectory generators. When 
 * the robot is pushed, the XY portion of the trajectory are replanned so the robot can recover from the push.
 */
public class TwoWaypointTrajectoryGeneratorWithPushRecovery implements PositionTrajectoryGenerator
{
   private static final boolean VISUALIZE = true;

   private final String namePostFix = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final BooleanYoVariable visualize;
   
   private final int numberOfBallsInBag = 30;
   private final BagOfBalls bagOfBalls;
   private double t0ForViz;
   private double tfForViz;
   private double tForViz;

   protected final EnumYoVariable<TrajectoryWaypointGenerationMethod> waypointGenerationMethod;

   private final DoubleProvider stepTimeProvider;
   private final PositionProvider[] positionSources = new PositionProvider[2];
   private final VectorProvider[] velocitySources = new VectorProvider[2];

   private final DoubleYoVariable stepTime;
   private final DoubleYoVariable timeIntoStep;
   private final DoubleYoVariable defaultGroundClearance;

   private final BooleanYoVariable setInitialSwingVelocityToZero;

   private final YoFramePoint desiredPosition;
   private final YoFrameVector desiredVelocity;
   private final YoFrameVector desiredAcceleration;

   protected TwoWaypointTrajectoryParameters trajectoryParameters;

   private final SmoothCartesianWaypointConnectorTrajectoryGenerator2D pushRecoveryTrajectoryGenerator;
   private final BooleanYoVariable hasReplanned;
   private final DoubleYoVariable initialTime;
   private final PositionTrajectoryGenerator nominalTrajectoryGenerator;

   private FramePoint nominalTrajectoryPosition;
   private FrameVector nominalTrajectoryVelocity;
   private FrameVector nominalTrajectoryAcceleration;

   private double timeRemaining;

   //	private final SoftTouchdownTrajectoryGenerator touchdownTrajectoryGenerator;

   public TwoWaypointTrajectoryGeneratorWithPushRecovery(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider stepTimeProvider,
         PositionProvider initialPositionProvider, VectorProvider initialVelocityProvider, PositionProvider finalPositionProvider,
         VectorProvider finalDesiredVelocityProvider, TrajectoryParametersProvider trajectoryParametersProvider, YoVariableRegistry parentRegistry,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, PositionTrajectoryGenerator nominalTrajectoryGenerator,
         WalkingControllerParameters walkingControllerParameters, boolean visualize)
   {
      registry = new YoVariableRegistry(namePrefix + namePostFix);
      parentRegistry.addChild(registry);

      setInitialSwingVelocityToZero = new BooleanYoVariable(namePrefix + "SetInitialSwingVelocityToZero", registry);
      setInitialSwingVelocityToZero.set(false);

      this.waypointGenerationMethod = new EnumYoVariable<TrajectoryWaypointGenerationMethod>(namePrefix + "WaypointGenerationMethod", registry,
            TrajectoryWaypointGenerationMethod.class);

      this.stepTimeProvider = stepTimeProvider;

      this.hasReplanned = new BooleanYoVariable(namePrefix + "HasReplanned", this.registry);
      this.hasReplanned.set(false);

      this.initialTime = new DoubleYoVariable(namePrefix + "InitialTime", this.registry);
      this.initialTime.set(0.0);

      this.nominalTrajectoryPosition = new FramePoint();
      this.nominalTrajectoryVelocity = new FrameVector();
      this.nominalTrajectoryAcceleration = new FrameVector();

      positionSources[0] = initialPositionProvider;
      positionSources[1] = finalPositionProvider;

      velocitySources[0] = initialVelocityProvider;
      velocitySources[1] = finalDesiredVelocityProvider;

      stepTime = new DoubleYoVariable(namePrefix + "StepTime", registry);
      stepTime.set(stepTimeProvider.getValue());

      timeIntoStep = new DoubleYoVariable(namePrefix + "TimeIntoStep", registry);

      defaultGroundClearance = new DoubleYoVariable(namePrefix + "DefaultGroundClearance", registry);
      defaultGroundClearance.set(SimpleTwoWaypointTrajectoryParameters.getDefaultGroundClearance());

      desiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", referenceFrame, registry);
      desiredVelocity = new YoFrameVector(namePrefix + "DesiredVelocity", referenceFrame, registry);
      desiredAcceleration = new YoFrameVector(namePrefix + "DesiredAcceleration", referenceFrame, registry);

      this.visualize = new BooleanYoVariable(namePrefix + "Visualize", registry);
      this.visualize.set(VISUALIZE);

      this.nominalTrajectoryGenerator = nominalTrajectoryGenerator;

      this.pushRecoveryTrajectoryGenerator = new SmoothCartesianWaypointConnectorTrajectoryGenerator2D(namePrefix + "PushRecoveryTrajectory", referenceFrame,
            this.initialTime.getDoubleValue(), stepTimeProvider, initialPositionProvider, finalPositionProvider, initialVelocityProvider, parentRegistry,
            dynamicGraphicObjectsListRegistry, walkingControllerParameters);
      
      this.bagOfBalls = new BagOfBalls(numberOfBallsInBag, 0.01, namePrefix + "SwingTrajectoryBagOfBalls", registry,
            dynamicGraphicObjectsListRegistry);
   }

   public void initialize()
   {
      timeRemaining = stepTimeProvider.getValue();

      pushRecoveryTrajectoryGenerator.setTimeIntoStep(stepTime.getDoubleValue() - timeRemaining);
      timeIntoStep.set(stepTime.getDoubleValue() - timeRemaining);
      pushRecoveryTrajectoryGenerator.initialize();
      
      if(VISUALIZE)
      {
         visualizeTrajectory();
      }
   }

   public void compute(double time)
   {
	   if(timeIntoStep.getDoubleValue()-time > 0.01)
	   {
		   throw new RuntimeException("The time going into the push recovery trajectory generator experienced a large jump.");
	   }
	   
      timeIntoStep.set(time);

      nominalTrajectoryGenerator.compute(time);
      pushRecoveryTrajectoryGenerator.compute(time);

      nominalTrajectoryGenerator.get(nominalTrajectoryPosition);
      nominalTrajectoryGenerator.packVelocity(nominalTrajectoryVelocity);
      nominalTrajectoryGenerator.packAcceleration(nominalTrajectoryAcceleration);

      desiredPosition.setX(pushRecoveryTrajectoryGenerator.getDesiredPosition().getX());
      desiredPosition.setY(pushRecoveryTrajectoryGenerator.getDesiredPosition().getY());
      desiredPosition.setZ(nominalTrajectoryPosition.getZ());

      desiredVelocity.setX(pushRecoveryTrajectoryGenerator.getDesiredVelocity().getX());
      desiredVelocity.setY(pushRecoveryTrajectoryGenerator.getDesiredVelocity().getY());
      desiredVelocity.setZ(nominalTrajectoryVelocity.getZ());

      desiredAcceleration.setX(pushRecoveryTrajectoryGenerator.getDesiredAcceleration().getX());
      desiredAcceleration.setY(pushRecoveryTrajectoryGenerator.getDesiredAcceleration().getY());
      desiredAcceleration.setZ(nominalTrajectoryAcceleration.getZ());

   }
   
   private void visualizeTrajectory()
   {
      t0ForViz = timeIntoStep.getDoubleValue();
      tfForViz = stepTime.getDoubleValue();

      for (int i = 0; i < numberOfBallsInBag; i++)
      {
         tForViz = t0ForViz + (double) i / (double) (numberOfBallsInBag) * (tfForViz - t0ForViz);
         computePositionsForVis(tForViz);
         bagOfBalls.setBall(desiredPosition.getFramePointCopy(), i);
      }
   }
   
   public void computePositionsForVis(double time)
   { 
      nominalTrajectoryGenerator.compute(time);
      pushRecoveryTrajectoryGenerator.compute(time);

      nominalTrajectoryGenerator.get(nominalTrajectoryPosition);
      nominalTrajectoryGenerator.packVelocity(nominalTrajectoryVelocity);
      nominalTrajectoryGenerator.packAcceleration(nominalTrajectoryAcceleration);

      desiredPosition.setX(pushRecoveryTrajectoryGenerator.getDesiredPosition().getX());
      desiredPosition.setY(pushRecoveryTrajectoryGenerator.getDesiredPosition().getY());
      desiredPosition.setZ(nominalTrajectoryPosition.getZ());
   }

   public void get(FramePoint positionToPack)
   {
      desiredPosition.getFrameTupleIncludingFrame(positionToPack);
   }

   public void packVelocity(FrameVector velocityToPack)
   {
      desiredVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   public void packAcceleration(FrameVector accelerationToPack)
   {
      desiredAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   @Override
   public boolean isDone()
   {
      return timeIntoStep.getDoubleValue() >= stepTime.getDoubleValue();
   }

   public void packLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      get(positionToPack);
      packVelocity(velocityToPack);
      packAcceleration(accelerationToPack);
   }
}
