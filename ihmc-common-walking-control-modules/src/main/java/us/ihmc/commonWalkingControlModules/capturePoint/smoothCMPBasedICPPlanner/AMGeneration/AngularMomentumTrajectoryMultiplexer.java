package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration;

import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPPointsInFoot;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

/**
 * This class is a wrapper around the predicted and commanded angular momentum trajectory generators.
 * If a reference trajectory is available in {@code commandedAngularMomentum}, it will be used. Otherwise
 * the predicted angular momentum is used.
 */
public class AngularMomentumTrajectoryMultiplexer implements AngularMomentumTrajectoryGeneratorInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean usingReferenceAngularMomentum = new YoBoolean("usingReferenceAngularMomentum", registry);

   /** Reference angular momentum trajectory supplied externally */
   private final CommandBasedAngularMomentumTrajectoryGenerator commandedAngularMomentum;
   /** Predicted angular momentum trajectory which is calculated based on upcoming footsteps */
   private final FootstepAngularMomentumPredictor predictedAngularMomentum;

   private AngularMomentumTrajectoryGeneratorInterface currentAngularMomentumTrajectoryGenerator;

   public AngularMomentumTrajectoryMultiplexer(String namePrefix, WalkingMessageHandler walkingMessageHandler, YoDouble yoTime, YoDouble omega0, boolean debug, YoVariableRegistry parentRegistry)
   {
      predictedAngularMomentum = new FootstepAngularMomentumPredictor(namePrefix, omega0, debug, registry);

      if(walkingMessageHandler == null)
      {
         commandedAngularMomentum = null;
      }
      else
      {
         commandedAngularMomentum = new CommandBasedAngularMomentumTrajectoryGenerator(walkingMessageHandler, yoTime, registry);
      }

      currentAngularMomentumTrajectoryGenerator = predictedAngularMomentum;
      usingReferenceAngularMomentum.set(false);

      parentRegistry.addChild(registry);
   }

   @Override
   public void clear()
   {
      predictedAngularMomentum.clear();

      if(commandedAngularMomentum != null)
      {
         commandedAngularMomentum.clear();
      }
   }

   @Override
   public void initializeParameters(SmoothCMPPlannerParameters smoothCMPPlannerParameters, double totalMass, double gravityZ)
   {
      predictedAngularMomentum.initializeParameters(smoothCMPPlannerParameters, totalMass, gravityZ);

      if(commandedAngularMomentum != null)
      {
         commandedAngularMomentum.initializeParameters(smoothCMPPlannerParameters, totalMass, gravityZ);
      }
   }

   @Override
   public void update(double currentTime)
   {
      currentAngularMomentumTrajectoryGenerator.update(currentTime);
   }

   @Override
   public void getDesiredAngularMomentum(FixedFrameVector3DBasics desiredAngMomToPack, FixedFrameVector3DBasics desiredTorqueToPack)
   {
      currentAngularMomentumTrajectoryGenerator.getDesiredAngularMomentum(desiredAngMomToPack, desiredTorqueToPack);
   }

   @Override
   public void initializeForDoubleSupport(double currentTime, boolean isStanding)
   {
      currentAngularMomentumTrajectoryGenerator.initializeForDoubleSupport(currentTime, isStanding);
   }

   @Override
   public void initializeForSingleSupport(double currentTime)
   {
      currentAngularMomentumTrajectoryGenerator.initializeForSingleSupport(currentTime);
   }

   @Override
   public void computeReferenceAngularMomentumStartingFromDoubleSupport(boolean atAStop)
   {
      updateCurrentAngularMomentumTrajectoryGenerator();
      currentAngularMomentumTrajectoryGenerator.computeReferenceAngularMomentumStartingFromDoubleSupport(atAStop);
   }

   @Override
   public void computeReferenceAngularMomentumStartingFromSingleSupport()
   {
      updateCurrentAngularMomentumTrajectoryGenerator();
      currentAngularMomentumTrajectoryGenerator.computeReferenceAngularMomentumStartingFromSingleSupport();
   }

   @Override
   public List<AngularMomentumTrajectory> getTransferAngularMomentumTrajectories()
   {
      return currentAngularMomentumTrajectoryGenerator.getTransferAngularMomentumTrajectories();
   }

   @Override
   public List<AngularMomentumTrajectory> getSwingAngularMomentumTrajectories()
   {
      return currentAngularMomentumTrajectoryGenerator.getSwingAngularMomentumTrajectories();
   }

   @Override
   public void addCopAndComSetpointsToPlan(List<CoPPointsInFoot> copLocations, List<? extends FramePoint3DReadOnly> comInitialPositions,
                                           List<? extends FramePoint3DReadOnly> comFinalPositions, List<? extends FrameVector3DReadOnly> comInitialVelocities,
                                           List<? extends FrameVector3DReadOnly> comFinalVelocities,
                                           List<? extends FrameVector3DReadOnly> comInitialAccelerations,
                                           List<? extends FrameVector3DReadOnly> comFinalAccelerations, int numberOfRegisteredFootsteps)
   {
      predictedAngularMomentum.addCopAndComSetpointsToPlan(copLocations, comInitialPositions, comFinalPositions, comInitialVelocities, comFinalVelocities,
                                                           comInitialAccelerations, comFinalAccelerations, numberOfRegisteredFootsteps);

      if (commandedAngularMomentum != null)
      {
         commandedAngularMomentum.addCopAndComSetpointsToPlan(copLocations, comInitialPositions, comFinalPositions, comInitialVelocities, comFinalVelocities,
                                                              comInitialAccelerations, comFinalAccelerations, numberOfRegisteredFootsteps);
      }
   }

   private void updateCurrentAngularMomentumTrajectoryGenerator()
   {
      if(commandedAngularMomentum != null && commandedAngularMomentum.hasReferenceTrajectory())
      {
         usingReferenceAngularMomentum.set(true);
         currentAngularMomentumTrajectoryGenerator = commandedAngularMomentum;
      }
      else
      {
         usingReferenceAngularMomentum.set(false);
         currentAngularMomentumTrajectoryGenerator = predictedAngularMomentum;
      }
   }
}
