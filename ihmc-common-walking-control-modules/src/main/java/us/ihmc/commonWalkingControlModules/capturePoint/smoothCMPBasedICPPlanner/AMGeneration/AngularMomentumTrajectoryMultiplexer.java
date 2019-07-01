package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration;

import java.util.List;

import controller_msgs.msg.dds.MomentumTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPPointsInFoot;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.messageHandlers.MomentumTrajectoryHandler;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This class is a wrapper around the predicted and commanded angular momentum trajectory generators.
 * If the controller receives a {@link MomentumTrajectoryMessage}, it will be used. Otherwise the predicted angular momentum is used.
 */
public class AngularMomentumTrajectoryMultiplexer implements AngularMomentumTrajectoryGeneratorInterface
{
   private static final int maxNumberOfStepsToConsider = 4;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoBoolean usingReferenceAngularMomentum = new YoBoolean("usingReferenceAngularMomentum", registry);

   /** Reference angular momentum trajectory supplied externally */
   private final CommandBasedAngularMomentumTrajectoryGenerator commandedAngularMomentum;
   /** Predicted angular momentum trajectory which is calculated based on upcoming footsteps */
   private final FootstepAngularMomentumPredictor predictedAngularMomentum;

   private AngularMomentumTrajectoryGeneratorInterface currentAngularMomentumTrajectoryGenerator;
   private boolean planSwingAngularMomentum;
   private boolean planTransferAngularMomentum;

   public AngularMomentumTrajectoryMultiplexer(String namePrefix, MomentumTrajectoryHandler momentumTrajectoryHandler, YoDouble yoTime, YoDouble omega0,
                                               boolean debug, YoVariableRegistry parentRegistry)
   {
      predictedAngularMomentum = new FootstepAngularMomentumPredictor(namePrefix, omega0, debug, maxNumberOfStepsToConsider, registry);

      if (momentumTrajectoryHandler == null)
      {
         commandedAngularMomentum = null;
      }
      else
      {
         commandedAngularMomentum = new CommandBasedAngularMomentumTrajectoryGenerator(momentumTrajectoryHandler, yoTime, maxNumberOfStepsToConsider, registry);
      }

      currentAngularMomentumTrajectoryGenerator = predictedAngularMomentum;
      usingReferenceAngularMomentum.set(false);

      parentRegistry.addChild(registry);
   }

   @Override
   public void clear()
   {
      predictedAngularMomentum.clear();

      if (commandedAngularMomentum != null)
      {
         commandedAngularMomentum.clear();
      }
   }

   public void initializeParameters(ICPPlannerParameters icpPlannerParameters, double totalMass, double gravityZ)
   {
      planSwingAngularMomentum = icpPlannerParameters.planSwingAngularMomentum();
      planTransferAngularMomentum = icpPlannerParameters.planTransferAngularMomentum();
      predictedAngularMomentum.initializeParameters(icpPlannerParameters, totalMass, gravityZ);

      if (commandedAngularMomentum != null)
      {
         commandedAngularMomentum.initializeParameters(icpPlannerParameters);
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
   public void computeReferenceAngularMomentumStartingFromDoubleSupport(boolean initialTransfer, boolean standing)
   {
      updateCurrentAngularMomentumTrajectoryGenerator();
      currentAngularMomentumTrajectoryGenerator.computeReferenceAngularMomentumStartingFromDoubleSupport(initialTransfer, standing);
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
      if (referenceTrajectoryIsAvailable())
      {
         usingReferenceAngularMomentum.set(true);
         currentAngularMomentumTrajectoryGenerator = commandedAngularMomentum;
      }
      else
      {
         // If we are switching from the commanded angular momentum to the prediction reset the commanded trajectory handler.
         if (usingReferenceAngularMomentum.getValue())
         {
            commandedAngularMomentum.reset();
         }

         usingReferenceAngularMomentum.set(false);
         currentAngularMomentumTrajectoryGenerator = predictedAngularMomentum;
      }
   }

   private boolean referenceTrajectoryIsAvailable()
   {
      return commandedAngularMomentum != null && commandedAngularMomentum.hasReferenceTrajectory();
   }

   public boolean isPredictingAngularMomentum()
   {
      boolean isPlanningAngularMomentum = planSwingAngularMomentum || planTransferAngularMomentum;
      boolean isUsingPrediction = !referenceTrajectoryIsAvailable();
      return isPlanningAngularMomentum && isUsingPrediction;
   }
}
