package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import com.jme3.terrain.geomipmap.UpdatedTerrainPatch;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.MomentumTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.CoPPointsInFoot;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.YoSegmentedFrameTrajectory3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.communication.packets.momentum.TrajectoryPoint3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class FootstepAngularMomentumEstimator implements AngularMomentumTrajectoryGeneratorInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final int maxNumberOfTrajectoryCoefficients = 4;
   private final int numberOfSwingSegments = 2;
   private final int numberOfTransferSegments = 2;

   private final YoInteger numberOfFootstepsToConsider;
   private final CoPPointName trajectoryPointStart;
   private final CoPPointName trajectoryPointEnd;
   private final CoPPointName trajectoryInitialDepartureReference;
   private final CoPPointName trajectoryFinalApproachReference;
   private final CoPPointName exitCoP;
   private final CoPPointName entryCoP;

   private final List<CoPPointsInFoot> upcomingCoPsInFootsteps;

   private final List<SwingAngularMomentumTrajectory> swingAngularMomentumTrajectories;
   private final List<TransferAngularMomentumTrajectory> transferAngularMomentumTrajectories;

   private final FrameVector desiredAngularMomentum = new FrameVector();
   private final FrameVector desiredTorque = new FrameVector();
   private final FrameVector desiredRotatum = new FrameVector();

   private final class CoMTrajectory extends YoSegmentedFrameTrajectory3D
   {
      public CoMTrajectory(String name, int maxNumberOfSegments, int maxNumberOfCoefficients, YoVariableRegistry registry)
      {
         super(name, maxNumberOfSegments, maxNumberOfCoefficients, registry);
      }

      public void setSegment(double t0, double tFinal, FramePoint z0, FramePoint zR1, FramePoint zR2, FramePoint zFinal)
      {
         segments.get(numberOfSegments.getIntegerValue()).setCubicBezier(t0, tFinal, z0, zR1, zR2, zFinal);
         numberOfSegments.increment();
      }

      public void trimSegment(int segmentIndex, double t0, double tFinal)
      {
         if(segments.get(segmentIndex).timeIntervalContains(t0))
            segments.get(segmentIndex).setInitialTime(t0);
         if(segments.get(segmentIndex).timeIntervalContains(tFinal))
            segments.get(segmentIndex).setFinalTime(tFinal);
      }
   }
   
   private final CoMTrajectory comTrajectory = new CoMTrajectory("TempCoMTrajectoryForAngularMomentumApproximation", Math.max(numberOfSwingSegments, numberOfTransferSegments), maxNumberOfTrajectoryCoefficients, registry);
   private final YoFrameTrajectory3D swingFootTrajectory;
   private final YoFrameTrajectory3D swingFootVelocity;
   private final YoFrameTrajectory3D angularMomentumTrajectory;
   private final FramePoint tempFramePoint1 = new FramePoint(), tempFramePoint2 = new FramePoint(), tempFramePoint3 = new FramePoint(),
         tempFramePoint4 = new FramePoint();

   private AngularMomentumTrajectoryInterface activeTrajectory;
   private double initialTime;
   private double currentStepSwingTime, currentStepTransferTime;
   private boolean tempFlag;

   public FootstepAngularMomentumEstimator(String namePrefix, AngularMomentumEstimationParameters angularMomentumParameters, YoVariableRegistry parentRegistry)
   {
      this.numberOfFootstepsToConsider = new YoInteger(namePrefix + "AngularMomentumPlanMaxFootsteps", registry);
      this.numberOfFootstepsToConsider.set(angularMomentumParameters.getNumberOfFootstepsToConsider());
      this.trajectoryPointStart = angularMomentumParameters.getInitialCoPPointName();
      this.trajectoryInitialDepartureReference = angularMomentumParameters.getInitialDepartureReferenceName();
      this.trajectoryPointEnd = angularMomentumParameters.getEndCoPName();
      this.trajectoryFinalApproachReference = angularMomentumParameters.getFinalApproachReferenceName();
      this.entryCoP = angularMomentumParameters.getEntryCoPName();
      this.exitCoP = angularMomentumParameters.getExitCoPName();

      this.swingAngularMomentumTrajectories = new ArrayList<>(numberOfFootstepsToConsider.getIntegerValue());
      this.transferAngularMomentumTrajectories = new ArrayList<>(numberOfFootstepsToConsider.getIntegerValue());
      this.upcomingCoPsInFootsteps = new ArrayList<>(numberOfFootstepsToConsider.getIntegerValue() + 2);

      ReferenceFrame[] referenceFrames = {worldFrame};
      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         SwingAngularMomentumTrajectory swingTrajectory = new SwingAngularMomentumTrajectory(namePrefix + "Footstep", i, registry, worldFrame,
                                                                                             numberOfSwingSegments);
         this.swingAngularMomentumTrajectories.add(swingTrajectory);
         TransferAngularMomentumTrajectory transferTrajectory = new TransferAngularMomentumTrajectory(namePrefix + "Footstep", i, registry, worldFrame,
                                                                                                      numberOfTransferSegments);
         this.transferAngularMomentumTrajectories.add(transferTrajectory);
         CoPPointsInFoot copLocations = new CoPPointsInFoot(i, referenceFrames, registry);
         upcomingCoPsInFootsteps.add(copLocations);
      }

      this.swingFootTrajectory = new YoFrameTrajectory3D("EstimatedCoMTrajectory", maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.swingFootVelocity = new YoFrameTrajectory3D("EstimatedCoMTrajectory", maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.angularMomentumTrajectory = new YoFrameTrajectory3D("EstiamtedAngularMomentumTrajectory", 7, worldFrame, registry);
      parentRegistry.addChild(registry);
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
      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         swingAngularMomentumTrajectories.get(i).reset();
         transferAngularMomentumTrajectories.get(i).reset();
      }
   }

   public void addFootstepCoPsToPlan(List<CoPPointsInFoot> copLocations)
   {
      for (int i = 0; i < copLocations.size(); i++)
         upcomingCoPsInFootsteps.get(i).setIncludingFrame(copLocations.get(i));
   }

   @Override
   public void update(double currentTime)
   {
      double timeInState = currentTime - initialTime;

      if (activeTrajectory != null)
         activeTrajectory.update(timeInState, desiredAngularMomentum, desiredTorque);
   }

   @Override
   public void getDesiredAngularMomentum(FrameVector desiredAngMomToPack)
   {
      desiredAngMomToPack.set(desiredAngularMomentum);
   }

   @Override
   public void getDesiredAngularMomentum(FrameVector desiredAngMomToPack, FrameVector desiredTorqueToPack)
   {
      desiredAngMomToPack.set(desiredAngularMomentum);
      desiredTorqueToPack.set(desiredTorque);
   }

   public void getDesiredAngularMomentum(FrameVector desiredAngMomToPack, FrameVector desiredTorqueToPack, FrameVector desiredRotatumToPack)
   {
      desiredAngMomToPack.set(desiredAngularMomentum);
      desiredTorqueToPack.set(desiredTorque);
      desiredRotatumToPack.set(desiredRotatum);
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
   }

   @Override
   public void initializeForSwing(double currentTime)
   {
      initialTime = currentTime;
      activeTrajectory = swingAngularMomentumTrajectories.get(0);
   }

   @Override
   public void computeReferenceAngularMomentumStartingFromDoubleSupport(boolean atAStop, RobotSide transferToSide)
   {
      int numberOfFootstepToPlan = Math.min(numberOfFootstepsToConsider.getIntegerValue(), upcomingCoPsInFootsteps.size());
      for (int index = 0; index < numberOfFootstepToPlan; index++)
         computeTransferAngularMomentumApproximationForFootstep(index);
   }

   private void computeTransferAngularMomentumApproximationForFootstep(int footstepIndex)
   {
      computeTransferTime(footstepIndex);
      setCoMTrajectoryForTransfer(footstepIndex);
      setSwingFootTrajectoryForTransfer(footstepIndex);

   }

   @Override
   public void computeReferenceAngularMomentumStartingFromSingleSupport(RobotSide supportSide)
   {
      int numberOfFootstepToPlan = Math.min(numberOfFootstepsToConsider.getIntegerValue(), upcomingCoPsInFootsteps.size());
      for (int index = 0; index < numberOfFootstepToPlan; index++)
         computeSwingAngularMomentumApproximationForFootstep(index);
   }

   private void computeSwingAngularMomentumApproximationForFootstep(int footstepIndex)
   {
      computeSwingTime(footstepIndex);
      setCoMTrajectoryForSwing(footstepIndex);
      setSwingFootTrajectoryForSwing(footstepIndex);

   }

   private void computeSwingTime(int footstepIndex)
   {
      this.currentStepSwingTime = 0.0;
      this.tempFlag = false;
      CoPPointsInFoot currentCoPPlan = upcomingCoPsInFootsteps.get(footstepIndex + 1);
      List<CoPPointName> copList = currentCoPPlan.getCoPPointList();
      for (int i = 0; i < copList.size(); i++)
      {
         if (tempFlag)
            this.currentStepSwingTime += currentCoPPlan.get(copList.get(i)).getTime();
         if (copList.get(i) == entryCoP)
            tempFlag = true;
         else if (copList.get(i) == exitCoP)
            break;
      }
   }

   private void computeTransferTime(int footstepIndex)
   {
      this.currentStepTransferTime = 0.0;
      this.tempFlag = false;
      CoPPointsInFoot currentCoPPlan = upcomingCoPsInFootsteps.get(footstepIndex);
      List<CoPPointName> copList = currentCoPPlan.getCoPPointList();
      for (int i = 0; i < copList.size(); i++)
      {
         if (tempFlag)
            this.currentStepTransferTime += currentCoPPlan.get(copList.get(i)).getTime();
         if (copList.get(i) == exitCoP)
            tempFlag = true;
      }
      currentCoPPlan = upcomingCoPsInFootsteps.get(footstepIndex + 1);
      copList = currentCoPPlan.getCoPPointList();
      for (int i = 0; i < copList.size(); i++)
      {
         this.currentStepTransferTime += currentCoPPlan.get(copList.get(i)).getTime();
         if (copList.get(i) == entryCoP)
            break;
      }
   }
   
   private void setSwingFootTrajectoryForSwing(int footstepIndex)
   {
      upcomingCoPsInFootsteps.get(footstepIndex).getFootLocation(tempFramePoint1);
      upcomingCoPsInFootsteps.get(footstepIndex + 1).getFootLocation(tempFramePoint2);
      swingFootTrajectory.setLinear(0.0, currentStepSwingTime, tempFramePoint1, tempFramePoint2);
      swingFootTrajectory.getDerivative(swingFootVelocity);
   }

   private void setSwingFootTrajectoryForTransfer(int footstepIndex)
   {
      upcomingCoPsInFootsteps.get(footstepIndex).getFootLocation(tempFramePoint1);
      swingFootTrajectory.setConstant(0.0, currentStepTransferTime, tempFramePoint1);
      swingFootTrajectory.getDerivative(swingFootVelocity);
   }

   private void setCoMTrajectoryForSwing(int footstepIndex)
   {
      upcomingCoPsInFootsteps.get(footstepIndex).get(trajectoryPointStart).getPosition(tempFramePoint1);
      upcomingCoPsInFootsteps.get(footstepIndex + 1).get(trajectoryInitialDepartureReference).getPosition(tempFramePoint2);
      upcomingCoPsInFootsteps.get(footstepIndex + 1).get(trajectoryFinalApproachReference).getPosition(tempFramePoint3);
      upcomingCoPsInFootsteps.get(footstepIndex + 1).get(trajectoryPointEnd).getPosition(tempFramePoint4);

      comTrajectory.setSegment(0.0, currentStepSwingTime, tempFramePoint1, tempFramePoint2, tempFramePoint3, tempFramePoint4);
      comTrajectory.trimSegment(comTrajectory.getNumberOfSegments() - 1, 0.0, currentStepSwingTime);
   }

   private void setCoMTrajectoryForTransfer(int footstepIndex)
   {
      upcomingCoPsInFootsteps.get(footstepIndex).get(trajectoryPointStart).getPosition(tempFramePoint1);
      upcomingCoPsInFootsteps.get(footstepIndex + 1).get(trajectoryInitialDepartureReference).getPosition(tempFramePoint2);
      upcomingCoPsInFootsteps.get(footstepIndex + 1).get(trajectoryFinalApproachReference).getPosition(tempFramePoint3);
      upcomingCoPsInFootsteps.get(footstepIndex + 1).get(trajectoryPointEnd).getPosition(tempFramePoint4);
      comTrajectory.setSegment(0.0, currentStepTransferTime, tempFramePoint1, tempFramePoint2, tempFramePoint3, tempFramePoint4);
   }

   @Override
   public List<? extends AngularMomentumTrajectoryInterface> getTransferCoPTrajectories()
   {
      return transferAngularMomentumTrajectories;
   }

   @Override
   public List<? extends AngularMomentumTrajectoryInterface> getSwingCoPTrajectories()
   {
      return swingAngularMomentumTrajectories;
   }
}
