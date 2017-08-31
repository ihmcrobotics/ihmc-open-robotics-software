package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.CoPPointsInFoot;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.WalkingTrajectoryType;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.TrajectoryMathTools;
import us.ihmc.robotics.math.trajectories.YoFrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.YoSegmentedFrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.YoTrajectory;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Estimates the angular momentum generated by the swing foot about the CoM during a footstep
 * Needs a footstep CoP plan. Uses the entry, exit and end CoPs defined in the CoP plan to calculate a segmented CoM trajectory
 * The CoM trajectory is then used along with the footstep plan to determine the angular momentum generated
 */
public class FootstepAngularMomentumPredictor implements AngularMomentumTrajectoryGeneratorInterface
{
   private static final boolean DEBUG = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final FrameVector3D zeroVector = new FrameVector3D();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final int maxNumberOfTrajectoryCoefficients = 7;
   private final int numberOfSwingSegments = 3;
   private final int numberOfTransferSegments = 2;
   private final int maxNumberOfFootstepsToConsider = 10;
   private final TrajectoryMathTools trajMathTools;

   private final YoBoolean computePredictedAngularMomentum;
   private final YoInteger numberOfFootstepsToConsider;
   private CoPPointName trajectoryInitialDepartureReference;
   private CoPPointName trajectoryFinalApproachReference;
   private CoPPointName entryCoPName;
   private CoPPointName exitCoPName;
   private CoPPointName endCoPName;

   private final YoDouble omega0;
   private final YoDouble gravityZ;
   private final YoDouble swingLegMass;
   private final YoDouble supportLegMass;
   private final YoDouble comHeight;
   private final YoDouble swingFootMaxHeight;

   private final List<CoPPointsInFoot> upcomingCoPsInFootsteps;
   private final YoInteger numberOfRegisteredFootsteps;

   private final List<SwingAngularMomentumTrajectory> swingAngularMomentumTrajectories;
   private final List<TransferAngularMomentumTrajectory> transferAngularMomentumTrajectories;

   private final FrameVector3D desiredAngularMomentum = new FrameVector3D();
   private final FrameVector3D desiredTorque = new FrameVector3D();
   private final FrameVector3D desiredRotatum = new FrameVector3D();

   //private final YoFrameTrajectory3D footstepCoMTrajectory;
   private final YoFrameTrajectory3D segmentCoMTrajectory;
   private final YoFrameTrajectory3D segmentCoMVelocity;
   private final YoFrameTrajectory3D phaseSwingFootTrajectory;
   private final YoFrameTrajectory3D segmentSwingFootTrajectory;
   private final YoTrajectory swingLiftTrajectory;
   private final YoFrameTrajectory3D swingFootVelocity;
   private final YoFrameTrajectory3D phaseSupportFootTrajectory;
   private final YoFrameTrajectory3D segmentSupportFootTrajectory;
   private final YoFrameTrajectory3D supportFootVelocity;
   private final YoFrameTrajectory3D estimatedAngularMomentumTrajectory;
   private AngularMomentumTrajectoryInterface activeTrajectory;
   private double initialTime;

   // DEBUGGING 
   private final YoFrameTrajectory3D swingTrajDebug;
   private final YoFrameTrajectory3D supportTrajDebug;
   private final YoFrameVector anguMomTrajDebug;
   private final YoFrameTrajectory3D comTrajDebug;
   private final YoFramePoint comPosDebug;
   private final YoFramePoint comVelDebug;
   private final YoFramePoint comAccDebug;
   private final YoFramePoint swingFootPosDebug;
   private final YoFramePoint swingFootVelDebug;
   private final YoFramePoint swingFootAccDebug;
   private final YoFramePoint supportFootPosDebug;
   private final YoFramePoint supportFootVelDebug;
   private final YoFramePoint supportFootAccDebug;
   private final List<TrajectoryDebug> transferCoMTrajectories;
   private final List<TrajectoryDebug> swingCoMTrajectories;
   private final List<TrajectoryDebug> transferSwingFootTrajectories;
   private final List<TrajectoryDebug> swingSwingFootTrajectories;
   private final List<TrajectoryDebug> transferSupportFootTrajectories;
   private final List<TrajectoryDebug> swingSupportFootTrajectories;
   private TrajectoryDebug activeCoMTrajectory;
   private TrajectoryDebug activeSwFootTrajectory;
   private TrajectoryDebug activeSpFootTrajectory;

   private final FramePoint3D tempFramePoint1 = new FramePoint3D();
   private final FramePoint3D tempFramePoint2 = new FramePoint3D();
   
   public FootstepAngularMomentumPredictor(String namePrefix, YoDouble omega0, YoVariableRegistry parentRegistry)
   {
      String fullPrefix = namePrefix + "AngularMomentum";
      this.trajMathTools = new TrajectoryMathTools(fullPrefix, 2 * maxNumberOfTrajectoryCoefficients, parentRegistry);
      this.computePredictedAngularMomentum = new YoBoolean(fullPrefix + "ComputePredictedAngularMomentum", registry);
      this.numberOfFootstepsToConsider = new YoInteger(fullPrefix + "MaxFootsteps", registry);
      this.omega0 = omega0;
      this.gravityZ = new YoDouble("AngularMomentumGravityZ", parentRegistry);
      omega0.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            comHeight.set(gravityZ.getDoubleValue() / (omega0.getDoubleValue() * omega0.getDoubleValue()));
         }
      });
      gravityZ.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            comHeight.set(gravityZ.getDoubleValue() / (omega0.getDoubleValue() * omega0.getDoubleValue()));
         }
      });
      this.swingLegMass = new YoDouble(fullPrefix + "SwingFootMass", registry);
      this.supportLegMass = new YoDouble(fullPrefix + "SupportFootMass", registry);
      this.comHeight = new YoDouble(fullPrefix + "CoMHeight", registry);

      this.swingFootMaxHeight = new YoDouble(fullPrefix + "SwingFootMaxHeight", registry);
      this.swingAngularMomentumTrajectories = new ArrayList<>(maxNumberOfFootstepsToConsider);
      this.transferAngularMomentumTrajectories = new ArrayList<>(maxNumberOfFootstepsToConsider + 1);

      if (DEBUG)
      {
         this.swingCoMTrajectories = new ArrayList<>(maxNumberOfFootstepsToConsider);
         this.transferCoMTrajectories = new ArrayList<>(maxNumberOfFootstepsToConsider + 1);
         this.swingSwingFootTrajectories = new ArrayList<>(maxNumberOfFootstepsToConsider);
         this.transferSwingFootTrajectories = new ArrayList<>(maxNumberOfFootstepsToConsider + 1);
         this.swingSupportFootTrajectories = new ArrayList<>(maxNumberOfFootstepsToConsider);
         this.transferSupportFootTrajectories = new ArrayList<>(maxNumberOfFootstepsToConsider + 1);
      }
      else
      {
         this.swingCoMTrajectories = null;
         this.transferCoMTrajectories = null;
         this.swingSwingFootTrajectories = null;
         this.transferSwingFootTrajectories = null;
         this.swingSupportFootTrajectories = null;
         this.transferSupportFootTrajectories = null;
      }

      this.upcomingCoPsInFootsteps = new ArrayList<>(maxNumberOfFootstepsToConsider + 2);
      this.numberOfRegisteredFootsteps = new YoInteger(fullPrefix + "NumberOfRegisteredFootsteps", registry);
      ReferenceFrame[] referenceFrames = {worldFrame};
      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         SwingAngularMomentumTrajectory swingTrajectory = new SwingAngularMomentumTrajectory(namePrefix + "Footstep", i, registry, worldFrame,
                                                                                             numberOfSwingSegments, 2 * maxNumberOfTrajectoryCoefficients);
         this.swingAngularMomentumTrajectories.add(swingTrajectory);

         TransferAngularMomentumTrajectory transferTrajectory = new TransferAngularMomentumTrajectory(namePrefix + "Footstep", i, registry, worldFrame,
                                                                                                      numberOfTransferSegments,
                                                                                                      2 * maxNumberOfTrajectoryCoefficients);
         this.transferAngularMomentumTrajectories.add(transferTrajectory);
         CoPPointsInFoot copLocations = new CoPPointsInFoot(i, referenceFrames, registry);
         upcomingCoPsInFootsteps.add(copLocations);

         if (DEBUG)
         {
            TrajectoryDebug swingCoMTrajectory = new TrajectoryDebug("SwingCoMTrajDebug" + i, numberOfSwingSegments, maxNumberOfTrajectoryCoefficients,
                                                                     registry);
            swingCoMTrajectories.add(swingCoMTrajectory);
            TrajectoryDebug transferCoMTrajectory = new TrajectoryDebug("TransferCoMTrajDebug" + i, numberOfTransferSegments, maxNumberOfTrajectoryCoefficients,
                                                                        registry);
            transferCoMTrajectories.add(transferCoMTrajectory);
            TrajectoryDebug swingSwingFootTrajectory = new TrajectoryDebug("SwingSwFootTrajDebug" + i, numberOfSwingSegments, maxNumberOfTrajectoryCoefficients,
                                                                           registry);
            swingSwingFootTrajectories.add(swingSwingFootTrajectory);
            TrajectoryDebug transferSwingFootTrajectory = new TrajectoryDebug("TransferSwFootTrajDebug" + i, numberOfTransferSegments,
                                                                              maxNumberOfTrajectoryCoefficients, registry);
            transferSwingFootTrajectories.add(transferSwingFootTrajectory);
            TrajectoryDebug swingSupportFootTrajectory = new TrajectoryDebug("SwingSpFootTrajDebug" + i, numberOfSwingSegments,
                                                                             maxNumberOfTrajectoryCoefficients, registry);
            swingSupportFootTrajectories.add(swingSupportFootTrajectory);
            TrajectoryDebug transferSupportFootTrajectory = new TrajectoryDebug("TransferSpFootTrajDebug" + i, numberOfTransferSegments,
                                                                                maxNumberOfTrajectoryCoefficients, registry);
            transferSupportFootTrajectories.add(transferSupportFootTrajectory);
         }
      }
      CoPPointsInFoot copLocations = new CoPPointsInFoot(maxNumberOfFootstepsToConsider, referenceFrames, registry);
      upcomingCoPsInFootsteps.add(copLocations);
      copLocations = new CoPPointsInFoot(maxNumberOfFootstepsToConsider + 1, referenceFrames, registry);
      upcomingCoPsInFootsteps.add(copLocations);
      TransferAngularMomentumTrajectory transferTrajectory = new TransferAngularMomentumTrajectory(namePrefix + "Footstep", maxNumberOfFootstepsToConsider,
                                                                                                   registry, worldFrame, numberOfTransferSegments,
                                                                                                   2 * maxNumberOfTrajectoryCoefficients);
      this.transferAngularMomentumTrajectories.add(transferTrajectory);
      if (DEBUG)
      {
         TrajectoryDebug transferCoMTrajectory = new TrajectoryDebug("TransferCoMTrajDebug" + maxNumberOfFootstepsToConsider, numberOfTransferSegments,
                                                                     maxNumberOfTrajectoryCoefficients, registry);
         transferCoMTrajectories.add(transferCoMTrajectory);
         TrajectoryDebug transferSwingFootTrajectory = new TrajectoryDebug("TransferSwFootTrajDebug" + maxNumberOfFootstepsToConsider, numberOfTransferSegments,
                                                                           maxNumberOfTrajectoryCoefficients, registry);
         transferSwingFootTrajectories.add(transferSwingFootTrajectory);
         TrajectoryDebug transferSupportFootTrajectory = new TrajectoryDebug("TransferSpFootTrajDebug" + maxNumberOfFootstepsToConsider,
                                                                             numberOfTransferSegments, maxNumberOfTrajectoryCoefficients, registry);
         transferSupportFootTrajectories.add(transferSupportFootTrajectory);
      }

      this.segmentCoMTrajectory = new YoFrameTrajectory3D(namePrefix + "EstSegmentTrajectory", maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.segmentCoMVelocity = new YoFrameTrajectory3D(namePrefix + "EstSegmentCoMVelocity", maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.phaseSwingFootTrajectory = new YoFrameTrajectory3D(namePrefix + "EstPhaseSwingTrajectory", 2 * maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.segmentSwingFootTrajectory = new YoFrameTrajectory3D(namePrefix + "EstSegmentSwingTrajectory", 2 * maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.swingFootVelocity = new YoFrameTrajectory3D(namePrefix + "EstSegmentSwingVelocity", maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.swingLiftTrajectory = new YoTrajectory(namePrefix + "SwingFootLiftTraj", maxNumberOfTrajectoryCoefficients, registry);
      this.phaseSupportFootTrajectory = new YoFrameTrajectory3D(namePrefix + "EstPhaseSupportTrajectory", 2 * maxNumberOfTrajectoryCoefficients, worldFrame,
                                                                registry);
      this.segmentSupportFootTrajectory = new YoFrameTrajectory3D(namePrefix + "EstSegmentSupportTrajectory", 2 * maxNumberOfTrajectoryCoefficients, worldFrame,
                                                                registry);
      this.supportFootVelocity = new YoFrameTrajectory3D(namePrefix + "EstSegmentSupportVelocity", maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.estimatedAngularMomentumTrajectory = new YoFrameTrajectory3D("EstSegmentAngularMomenetum", 2 * maxNumberOfTrajectoryCoefficients, worldFrame,
                                                                        registry);
      if (DEBUG)
      {
         this.anguMomTrajDebug = new YoFrameVector("AngMomViz", worldFrame, registry);
         this.comTrajDebug = new YoFrameTrajectory3D("CoMDebugTraj", 2 * maxNumberOfTrajectoryCoefficients, worldFrame, registry);
         this.swingTrajDebug = new YoFrameTrajectory3D("SwingDebugTraj", 2 * maxNumberOfTrajectoryCoefficients, worldFrame, registry);
         this.supportTrajDebug = new YoFrameTrajectory3D("SupportDebugTraj", 2 * maxNumberOfTrajectoryCoefficients, worldFrame, registry);
         this.comPosDebug = new YoFramePoint("CoMPosViz", "", worldFrame, registry);
         this.comVelDebug = new YoFramePoint("CoMVelViz", "", worldFrame, registry);
         this.comAccDebug = new YoFramePoint("CoMAccViz", "", worldFrame, registry);
         this.swingFootPosDebug = new YoFramePoint("SwFPosViz", worldFrame, registry);
         this.swingFootVelDebug = new YoFramePoint("SwFVelViz", worldFrame, registry);
         this.swingFootAccDebug = new YoFramePoint("SwFAccViz", worldFrame, registry);
         this.supportFootPosDebug = new YoFramePoint("SpFPosViz", worldFrame, registry);
         this.supportFootVelDebug = new YoFramePoint("SpFVelViz", worldFrame, registry);
         this.supportFootAccDebug = new YoFramePoint("SpFAccViz", worldFrame, registry);
      }
      else
      {
         comTrajDebug = null;
         swingTrajDebug = null;
         anguMomTrajDebug = null;
         comPosDebug = null;
         comVelDebug = null;
         comAccDebug = null;
         swingFootPosDebug = null;
         swingFootVelDebug = null;
         swingFootAccDebug = null;
         supportFootPosDebug = null;
         supportFootVelDebug = null;
         supportFootAccDebug = null;
      }

      parentRegistry.addChild(registry);
   }

   public void initializeParameters(AngularMomentumEstimationParameters angularMomentumParameters)
   {
      this.computePredictedAngularMomentum.set(angularMomentumParameters.computePredictedAngularMomentum());
      this.numberOfFootstepsToConsider.set(angularMomentumParameters.getNumberOfFootstepsToConsider());
      this.trajectoryInitialDepartureReference = angularMomentumParameters.getInitialDepartureReferenceName();
      this.trajectoryFinalApproachReference = angularMomentumParameters.getFinalApproachReferenceName();
      this.entryCoPName = angularMomentumParameters.getEntryCoPName();
      this.exitCoPName = angularMomentumParameters.getExitCoPName();
      this.endCoPName = angularMomentumParameters.getEndCoPName();
      this.swingLegMass.set(angularMomentumParameters.getSwingLegMass());
      this.supportLegMass.set(angularMomentumParameters.getSupportLegMass());
      this.swingFootMaxHeight.set(angularMomentumParameters.getSwingFootMaxLift());
      this.gravityZ.set(angularMomentumParameters.getGravityZ());
   }

   @Override
   public void updateListeners()
   {
      // TODO Auto-generated method stub      
   }

   @Override
   public void createVisualizerForConstantAngularMomentum(YoGraphicsList yoGraphicsList, ArtifactList artifactList)
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void clear()
   {
      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         swingAngularMomentumTrajectories.get(i).reset();
         transferAngularMomentumTrajectories.get(i).reset();
         if (DEBUG)
         {
            swingCoMTrajectories.get(i).reset();
            transferCoMTrajectories.get(i).reset();
            swingSwingFootTrajectories.get(i).reset();
            transferSwingFootTrajectories.get(i).reset();
            swingSupportFootTrajectories.get(i).reset();
            transferSupportFootTrajectories.get(i).reset();
         }
      }
      if (DEBUG)
      {
         transferCoMTrajectories.get(maxNumberOfFootstepsToConsider).reset();
         transferSwingFootTrajectories.get(maxNumberOfFootstepsToConsider).reset();
         transferSupportFootTrajectories.get(maxNumberOfFootstepsToConsider).reset();
      }
      transferAngularMomentumTrajectories.get(maxNumberOfFootstepsToConsider).reset();
      for (int i = 0; i < upcomingCoPsInFootsteps.size(); i++)
         upcomingCoPsInFootsteps.get(i).reset();
   }

   private List<FramePoint3D> comInitialPositions;
   private List<FramePoint3D> comFinalPositions;
   private List<FrameVector3D> comInitialVelocities;
   private List<FrameVector3D> comFinalVelocities;
   private List<FrameVector3D> comInitialAccelerations;
   private List<FrameVector3D> comFinalAccelerations;

   public void addFootstepCoPsToPlan(List<CoPPointsInFoot> copLocations, List<FramePoint3D> comInitialPositions, List<FramePoint3D> comFinalPositions,
                                     List<FrameVector3D> comInitialVelocities, List<FrameVector3D> comFinalVelocities,
                                     List<FrameVector3D> comInitialAccelerations, List<FrameVector3D> comFinalAccelerations, int numberOfRegisteredFootsteps)
   {
      for (int i = 0; i < copLocations.size(); i++)
         upcomingCoPsInFootsteps.get(i).setIncludingFrame(copLocations.get(i));
      this.numberOfRegisteredFootsteps.set(numberOfRegisteredFootsteps);
      this.comInitialPositions = comInitialPositions;
      this.comFinalPositions = comFinalPositions;
      this.comInitialVelocities = comInitialVelocities;
      this.comFinalVelocities = comFinalVelocities;
      this.comInitialAccelerations = comInitialAccelerations;
      this.comFinalAccelerations = comFinalAccelerations;
   }

   @Override
   public void update(double currentTime)
   {
      if (activeTrajectory != null && computePredictedAngularMomentum.getBooleanValue())
         activeTrajectory.update(currentTime - initialTime, desiredAngularMomentum, desiredTorque, desiredRotatum);
      else
      {
         desiredAngularMomentum.setToZero();
         desiredTorque.setToZero();
         desiredRotatum.setToZero();
      }
      getPredictedCenterOfMassPosition(currentTime);
      getPredictedSwingFootPosition(currentTime);
   }

   @Override
   public void getDesiredAngularMomentum(FrameVector3D desiredAngMomToPack)
   {
      desiredAngMomToPack.setIncludingFrame(desiredAngularMomentum);
   }

   @Override
   public void getDesiredAngularMomentum(FrameVector3D desiredAngMomToPack, FrameVector3D desiredTorqueToPack)
   {
      desiredAngMomToPack.setIncludingFrame(desiredAngularMomentum);
      desiredTorqueToPack.setIncludingFrame(desiredTorque);
   }

   public void getDesiredAngularMomentum(FrameVector3D desiredAngMomToPack, FrameVector3D desiredTorqueToPack, FrameVector3D desiredRotatumToPack)
   {
      desiredAngMomToPack.setIncludingFrame(desiredAngularMomentum);
      desiredTorqueToPack.setIncludingFrame(desiredTorque);
      desiredRotatumToPack.setIncludingFrame(desiredRotatum);
   }

   @Override
   public void getDesiredAngularMomentum(YoFrameVector desiredAngMomToPack)
   {
      desiredAngMomToPack.set(desiredAngularMomentum);
   }

   @Override
   public void getDesiredAngularMomentum(YoFrameVector desiredAngMomToPack, YoFrameVector desiredTorqueToPack)
   {
      desiredAngMomToPack.set(desiredAngularMomentum);
      desiredTorqueToPack.set(desiredTorque);
   }

   public void getDesiredAngularMomentum(YoFrameVector desiredAngMomToPack, YoFrameVector desiredTorqueToPack, YoFrameVector desiredRotatumToPack)
   {
      desiredAngMomToPack.set(desiredAngularMomentum);
      desiredTorqueToPack.set(desiredTorque);
      desiredRotatumToPack.set(desiredRotatum);
   }

   @Override
   public void initializeForTransfer(double currentTime)
   {
      initialTime = currentTime;
      activeTrajectory = transferAngularMomentumTrajectories.get(0);
      if (DEBUG)
      {
         activeCoMTrajectory = transferCoMTrajectories.get(0);
         activeSwFootTrajectory = transferSwingFootTrajectories.get(0);
         activeSpFootTrajectory = transferSupportFootTrajectories.get(0);
      }
   }

   @Override
   public void initializeForSwing(double currentTime)
   {
      initialTime = currentTime;
      activeTrajectory = swingAngularMomentumTrajectories.get(0);
      if (DEBUG)
      {
         activeCoMTrajectory = swingCoMTrajectories.get(0);
         activeSwFootTrajectory = swingSwingFootTrajectories.get(0);
         activeSpFootTrajectory = swingSupportFootTrajectories.get(0);
      }
   }

   @Override
   public void computeReferenceAngularMomentumStartingFromDoubleSupport(boolean atAStop)
   {
      int footstepIndex = 0;
      setAngularMomentumTrajectoryForFootsteps(footstepIndex, WalkingTrajectoryType.TRANSFER);
   }

   @Override
   public void computeReferenceAngularMomentumStartingFromSingleSupport()
   {
      int footstepIndex = 0;
      setAngularMomentumTrajectoryForFootsteps(footstepIndex, WalkingTrajectoryType.SWING);
   }

   private void setAngularMomentumTrajectoryForFootsteps(int footstepIndex, WalkingTrajectoryType initialWalkingPhase)
   {
      CoPPointsInFoot copPointsInFoot = upcomingCoPsInFootsteps.get(footstepIndex);
      double phaseTime = 0.0;
      int comIndex = 0;
      if(initialWalkingPhase == WalkingTrajectoryType.SWING)
      {
         copPointsInFoot = upcomingCoPsInFootsteps.get(footstepIndex + 1);
         setFootTrajectoriesForPhase(footstepIndex, initialWalkingPhase);
         for(int j = CoPPlanningTools.getCoPPointIndex(copPointsInFoot.getCoPPointList(), entryCoPName) + 1; j < copPointsInFoot.getNumberOfCoPPoints(); j++, comIndex++)
         {
            setCoMTrajectory(phaseTime, phaseTime + copPointsInFoot.get(j).getTime(), comIndex);
            setFootTrajectoriesForSegment(phaseTime, phaseTime + copPointsInFoot.get(j).getTime());
            if(DEBUG)
            {
               swingCoMTrajectories.get(footstepIndex).set(segmentCoMTrajectory);
               swingSwingFootTrajectories.get(footstepIndex).set(segmentSwingFootTrajectory);
               swingSupportFootTrajectories.get(footstepIndex).set(segmentSupportFootTrajectory);
            }
            calculateAngularMomentumTrajectory();
            swingAngularMomentumTrajectories.get(footstepIndex).set(estimatedAngularMomentumTrajectory);
            phaseTime += copPointsInFoot.get(j).getTime();
         }
         footstepIndex++;
         phaseTime = 0.0;
      }
      WalkingTrajectoryType currentWalkingPhase = WalkingTrajectoryType.TRANSFER;
      int numberOfSteps = Math.min(numberOfRegisteredFootsteps.getIntegerValue(), numberOfFootstepsToConsider.getIntegerValue());
      for(int i = footstepIndex; i < numberOfSteps; i++)
      {
         copPointsInFoot = upcomingCoPsInFootsteps.get(i + 1);
         setFootTrajectoriesForPhase(i, currentWalkingPhase);
         for(int j = 0; j < copPointsInFoot.getNumberOfCoPPoints(); j++, comIndex++)
         {
            setCoMTrajectory(phaseTime, phaseTime + copPointsInFoot.get(j).getTime(), comIndex);
            setFootTrajectoriesForSegment(phaseTime, phaseTime + copPointsInFoot.get(j).getTime());
            if(DEBUG)
            {
               if(currentWalkingPhase == WalkingTrajectoryType.TRANSFER)
               {
                  transferCoMTrajectories.get(i).set(segmentCoMTrajectory);
                  transferSwingFootTrajectories.get(i).set(segmentSwingFootTrajectory);
                  transferSupportFootTrajectories.get(i).set(segmentSupportFootTrajectory);
               }
               else if(currentWalkingPhase == WalkingTrajectoryType.SWING)
               {
                  swingCoMTrajectories.get(i).set(segmentCoMTrajectory);
                  swingSwingFootTrajectories.get(i).set(segmentSwingFootTrajectory);
                  swingSupportFootTrajectories.get(i).set(segmentSupportFootTrajectory);
               }
            }
            calculateAngularMomentumTrajectory();
            if(currentWalkingPhase == WalkingTrajectoryType.TRANSFER)
               transferAngularMomentumTrajectories.get(i).set(estimatedAngularMomentumTrajectory);
            else if(currentWalkingPhase == WalkingTrajectoryType.SWING)
               swingAngularMomentumTrajectories.get(i).set(estimatedAngularMomentumTrajectory);
            phaseTime += copPointsInFoot.get(j).getTime();
            if(copPointsInFoot.getCoPPointList().get(j) == entryCoPName)
            {
               currentWalkingPhase = WalkingTrajectoryType.SWING;
               phaseTime = 0.0;
            }
            else if(j >= copPointsInFoot.getNumberOfCoPPoints() - 1)
            {
               currentWalkingPhase = WalkingTrajectoryType.TRANSFER;
               phaseTime = 0.0;
            }
         }
      }
      copPointsInFoot = upcomingCoPsInFootsteps.get(numberOfSteps + 1);
      setFootTrajectoriesForPhase(numberOfSteps, currentWalkingPhase);
      for(int j = 0; j < copPointsInFoot.getNumberOfCoPPoints(); j++, comIndex++)
      {
         setCoMTrajectory(phaseTime, phaseTime + copPointsInFoot.get(j).getTime(), comIndex);
         setFootTrajectoriesForSegment(phaseTime, phaseTime + copPointsInFoot.get(j).getTime());
         if(DEBUG)
         {
               transferCoMTrajectories.get(numberOfSteps).set(segmentCoMTrajectory);
               transferSwingFootTrajectories.get(numberOfSteps).set(segmentSwingFootTrajectory);
               transferSupportFootTrajectories.get(numberOfSteps).set(segmentSupportFootTrajectory);
         }
         calculateAngularMomentumTrajectory();
         transferAngularMomentumTrajectories.get(numberOfSteps).set(estimatedAngularMomentumTrajectory);
         phaseTime += copPointsInFoot.get(j).getTime();
      }
   }
   
   private void setCoMTrajectory(double initialTime, double finalTime, int comIndex)
   {
      tempFramePoint1.set(comInitialPositions.get(comIndex));
      tempFramePoint1.addZ(comHeight.getDoubleValue());
      tempFramePoint2.set(comFinalPositions.get(comIndex));
      tempFramePoint2.addZ(comHeight.getDoubleValue());
      segmentCoMTrajectory.setQuintic(initialTime, finalTime, tempFramePoint1, comInitialVelocities.get(comIndex),
                                      comInitialAccelerations.get(comIndex), tempFramePoint2, comFinalVelocities.get(comIndex),
                                      comFinalAccelerations.get(comIndex));
      trajMathTools.getDerivative(segmentCoMVelocity, segmentCoMTrajectory);
   }

   
   private void setFootTrajectoriesForSegment(double initialTime, double finalTime)
   {
      segmentSwingFootTrajectory.set(phaseSwingFootTrajectory);
      segmentSwingFootTrajectory.setTime(initialTime, finalTime);
      trajMathTools.getDerivative(swingFootVelocity, segmentSwingFootTrajectory);
      segmentSupportFootTrajectory.set(phaseSupportFootTrajectory);
      segmentSupportFootTrajectory.setTime(initialTime, finalTime);
      trajMathTools.getDerivative(supportFootVelocity, segmentSupportFootTrajectory);
   }
   
   private void setFootTrajectoriesForPhase(int footstepIndex, WalkingTrajectoryType phase)
   {
      CoPPointsInFoot copPointsInFoot = upcomingCoPsInFootsteps.get(footstepIndex + 1);
      double phaseDuration = 0.0;
      if(phase == WalkingTrajectoryType.SWING)
      {
         for(int j = CoPPlanningTools.getCoPPointIndex(copPointsInFoot.getCoPPointList(), entryCoPName) + 1; j < copPointsInFoot.getNumberOfCoPPoints(); j++)
            phaseDuration += copPointsInFoot.get(j).getTime();
      }
      else
      {
         int j = 0;
         for(; j < copPointsInFoot.getNumberOfCoPPoints() - 1 && copPointsInFoot.getCoPPointList().get(j) != entryCoPName && copPointsInFoot.getCoPPointList().get(j) != CoPPointName.FINAL_COP; j++)
            phaseDuration += copPointsInFoot.get(j).getTime();
         phaseDuration += copPointsInFoot.get(j).getTime();
      }
      setSwingFootTrajectoryForPhase(footstepIndex, phase, phaseDuration);
      setSupportFootTrajectoryForPhase(footstepIndex, phase, phaseDuration);
   }
   
   private void setSwingFootTrajectoryForPhase(int footstepIndex, WalkingTrajectoryType phase, double phaseDuration)
   {
      if(phase == WalkingTrajectoryType.SWING)
      {
         upcomingCoPsInFootsteps.get(footstepIndex).getSupportFootLocation(tempFramePoint1);
         upcomingCoPsInFootsteps.get(footstepIndex + 1).getSwingFootLocation(tempFramePoint2);
         phaseSwingFootTrajectory.setPenticWithZeroTerminalAcceleration(0.0, phaseDuration, tempFramePoint1, zeroVector, tempFramePoint2, zeroVector);
      }
      else
      {
         upcomingCoPsInFootsteps.get(footstepIndex).getSupportFootLocation(tempFramePoint1);
         phaseSwingFootTrajectory.setPenticWithZeroTerminalAcceleration(0.0, phaseDuration, tempFramePoint1, zeroVector, tempFramePoint1, zeroVector);
      }
   }
   
   private void setSupportFootTrajectoryForPhase(int footstepIndex, WalkingTrajectoryType phase, double phaseDuration)
   {
      upcomingCoPsInFootsteps.get(footstepIndex + 1).getSupportFootLocation(tempFramePoint1);
      phaseSupportFootTrajectory.setPenticWithZeroTerminalAcceleration(0.0, phaseDuration, tempFramePoint1, zeroVector, tempFramePoint1, zeroVector);
   }

   private void calculateAngularMomentumTrajectory()
   {
      trajMathTools.subtract(segmentSwingFootTrajectory, segmentSwingFootTrajectory, segmentCoMTrajectory);
      trajMathTools.subtract(swingFootVelocity, swingFootVelocity, segmentCoMVelocity);
      trajMathTools.subtract(segmentSupportFootTrajectory, segmentSupportFootTrajectory, segmentCoMTrajectory);
      trajMathTools.subtract(supportFootVelocity, supportFootVelocity, segmentCoMVelocity);

      trajMathTools.crossProduct(segmentSwingFootTrajectory, segmentSwingFootTrajectory, swingFootVelocity);
      trajMathTools.scale(segmentSwingFootTrajectory, segmentSwingFootTrajectory, swingLegMass.getDoubleValue());
      trajMathTools.crossProduct(segmentSupportFootTrajectory, segmentSupportFootTrajectory, supportFootVelocity);
      trajMathTools.scale(segmentSupportFootTrajectory, segmentSupportFootTrajectory, supportLegMass.getDoubleValue());

      trajMathTools.add(estimatedAngularMomentumTrajectory, segmentSupportFootTrajectory, segmentSwingFootTrajectory);
   }

   FramePoint3D tempFramePointForCoMPosDebug = new FramePoint3D();
   FrameVector3D tempFramePointForCoMVelDebug = new FrameVector3D();
   FrameVector3D tempFramePointForCoMAccDebug = new FrameVector3D();
   
   public void getPredictedCenterOfMassPosition(YoFramePoint pointToPack, double time)
   {
      if (DEBUG && computePredictedAngularMomentum.getBooleanValue())
      {
         activeCoMTrajectory.update(time - initialTime, tempFramePointForCoMPosDebug, tempFramePointForCoMVelDebug, tempFramePointForCoMAccDebug);
         pointToPack.set(tempFramePointForCoMPosDebug);
         comPosDebug.set(tempFramePointForCoMPosDebug);
         comVelDebug.set(tempFramePointForCoMVelDebug);
         comAccDebug.set(tempFramePointForCoMAccDebug);
      }
   }

   public void getPredictedCenterOfMassPosition(double time)
   {
      if (DEBUG && computePredictedAngularMomentum.getBooleanValue())
      {
         activeCoMTrajectory.update(time - initialTime, tempFramePointForCoMPosDebug, tempFramePointForCoMVelDebug, tempFramePointForCoMAccDebug);
         comPosDebug.set(tempFramePointForCoMPosDebug);
         comVelDebug.set(tempFramePointForCoMVelDebug);
         comAccDebug.set(tempFramePointForCoMAccDebug);
      }
   }

   FramePoint3D tempFramePointForSwFootPosDebug = new FramePoint3D();
   FrameVector3D tempFramePointForSwFootVelDebug = new FrameVector3D();
   FrameVector3D tempFramePointForSwFootAccDebug = new FrameVector3D();
   FramePoint3D tempFramePointForSpFootPosDebug = new FramePoint3D();
   FrameVector3D tempFramePointForSpFootVelDebug = new FrameVector3D();
   FrameVector3D tempFramePointForSpFootAccDebug = new FrameVector3D();
   
   public void getPredictedFootPosition(YoFramePoint pointToPack, double time)
   {
      if (DEBUG && computePredictedAngularMomentum.getBooleanValue())
      {
         activeSwFootTrajectory.update(time - initialTime, tempFramePointForSwFootPosDebug, tempFramePointForSwFootVelDebug, tempFramePointForSwFootAccDebug);
         pointToPack.set(tempFramePointForSwFootPosDebug);
         swingFootPosDebug.set(tempFramePointForSwFootPosDebug);
         swingFootVelDebug.set(tempFramePointForSwFootVelDebug);
         swingFootAccDebug.set(tempFramePointForSwFootAccDebug);
         activeSpFootTrajectory.update(time - initialTime, tempFramePointForSpFootPosDebug, tempFramePointForSpFootVelDebug, tempFramePointForSpFootAccDebug);
         supportFootPosDebug.set(tempFramePointForSpFootPosDebug);
         supportFootVelDebug.set(tempFramePointForSpFootVelDebug);
         supportFootAccDebug.set(tempFramePointForSpFootAccDebug);
      }
   }

   public void getPredictedSwingFootPosition(double time)
   {
      if (DEBUG && computePredictedAngularMomentum.getBooleanValue())
      {
         activeSwFootTrajectory.update(time - initialTime, tempFramePointForSwFootPosDebug, tempFramePointForSwFootVelDebug, tempFramePointForSwFootAccDebug);
         swingFootPosDebug.set(tempFramePointForSwFootPosDebug);
         swingFootVelDebug.set(tempFramePointForSwFootVelDebug);
         swingFootAccDebug.set(tempFramePointForSwFootAccDebug);
         activeSpFootTrajectory.update(time - initialTime, tempFramePointForSpFootPosDebug, tempFramePointForSpFootVelDebug, tempFramePointForSpFootAccDebug);
         supportFootPosDebug.set(tempFramePointForSpFootPosDebug);
         supportFootVelDebug.set(tempFramePointForSpFootVelDebug);
         supportFootAccDebug.set(tempFramePointForSpFootAccDebug);
      }
   }

   @Override
   public List<? extends AngularMomentumTrajectory> getTransferAngularMomentumTrajectories()
   {
      if (computePredictedAngularMomentum.getBooleanValue())
         return transferAngularMomentumTrajectories;
      else
         return null;
   }

   @Override
   public List<? extends AngularMomentumTrajectory> getSwingAngularMomentumTrajectories()
   {
      if (computePredictedAngularMomentum.getBooleanValue())
         return swingAngularMomentumTrajectories;
      else
         return null;
   }

   private class TrajectoryDebug extends YoSegmentedFrameTrajectory3D
   {

      public TrajectoryDebug(String name, int maxNumberOfSegments, int maxNumberOfCoefficients, YoVariableRegistry registry)
      {
         super(name, maxNumberOfSegments, maxNumberOfCoefficients, registry);
      }

      public void set(YoFrameTrajectory3D trajToCopy)
      {
         segments.get(getNumberOfSegments()).set(trajToCopy);
         numberOfSegments.increment();
      }
   }
}
