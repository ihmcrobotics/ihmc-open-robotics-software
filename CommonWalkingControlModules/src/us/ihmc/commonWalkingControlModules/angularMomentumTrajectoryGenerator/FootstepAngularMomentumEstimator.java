package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.CoPPointsInFoot;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.YoFrameTrajectory3D;
import us.ihmc.robotics.trajectories.YoTrajectory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Estimates the angular momentum generated by the swing foot about the CoM during a footstep
 * Needs a footstep CoP plan. Uses the entry, exit and end CoPs defined in the CoP plan to calculate a segmented CoM trajectory
 * The CoM trajectory is then used along with the footstep plan to determine the angular momentum generated
 *
 */
public class FootstepAngularMomentumEstimator implements AngularMomentumTrajectoryGeneratorInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final FrameVector zeroVector = new FrameVector();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final int maxNumberOfTrajectoryCoefficients = 6;
   private final int numberOfSwingSegments = 1;
   private final int numberOfTransferSegments = 2;
   private final int maxNumberOfFootstepsToConsider = 10;

   private final YoInteger numberOfFootstepsToConsider;
   private CoPPointName trajectoryPointStart;
   private CoPPointName trajectoryPointEnd;
   private CoPPointName trajectoryInitialDepartureReference;
   private CoPPointName trajectoryFinalApproachReference;
   private CoPPointName exitCoP;
   private CoPPointName entryCoP;
   private CoPPointName endCoP;
   private CoPPointName[] transferCoPList;
   private final YoDouble swingLegMass;
   private final YoDouble supportLegMass;
   private final YoDouble comHeight;
   private final YoDouble swingFootMaxHeight;

   private final List<CoPPointsInFoot> upcomingCoPsInFootsteps;

   private final List<SwingAngularMomentumTrajectory> swingAngularMomentumTrajectories;
   private final List<TransferAngularMomentumTrajectory> transferAngularMomentumTrajectories;

   private final FrameVector desiredAngularMomentum = new FrameVector();
   private final FrameVector desiredTorque = new FrameVector();
   private final FrameVector desiredRotatum = new FrameVector();

   //private final YoFrameTrajectory3D footstepCoMTrajectory;
   private final YoFrameTrajectory3D segmentCoMTrajectory;
   private final YoFrameTrajectory3D segmentCoMVelocity;
   private final YoFrameTrajectory3D swingFootTrajectory;
   private final YoTrajectory swingLiftTrajectory;
   private final YoFrameTrajectory3D swingFootVelocity;
   private final YoFrameTrajectory3D supportFootTrajectory;
   private final YoFrameTrajectory3D supportFootVelocity;
   private final YoFrameTrajectory3D estimatedAngularMomentumTrajectory;
   private final YoFrameTrajectory3D previousEstimatedTransferTrajectory; // needed to compute the first double support trajectory segment 
   private final FramePoint previousFinalReferenceCoP = new FramePoint();
   private AngularMomentumTrajectoryInterface activeTrajectory;
   private double initialTime;
   private double previousFirstTransferEndTime;
   private double currentFootstepTime;
   private double currentSwingSegmentDuration;
   private double currentFirstTransferSegmentDuration;
   private double currentSecondTransferSegmentDuration;

   private enum TrajectorySegment
   {
      END_TO_ENTRY, EXIT_TO_END, ENTRY_TO_EXIT
   };

   private final FramePoint tempFramePoint1 = new FramePoint(), tempFramePoint2 = new FramePoint(), tempFramePoint3 = new FramePoint(), tempFramePoint4 = new FramePoint();
   private final FrameVector tempFrameVector1 = new FrameVector(), tempFrameVector2 = new FrameVector();
   private int tempInt1, tempInt2;
   private int footstepIndex;
   private double tempDouble;
   private CoPPointsInFoot currentCoPPlanReference;
   private List<CoPPointName> currentCoPListReference;

   // DEBUGGING 
   private boolean firstCoM, firstSwing, swingStart;
   private YoFrameTrajectory3D comTrajD;
   private YoFrameTrajectory3D swingTrajD;
   private YoFrameVector anguMomTraj;
   private YoFramePoint comPos;
   private YoFramePoint swingFootTraj;

   public FootstepAngularMomentumEstimator(String namePrefix, YoVariableRegistry parentRegistry)
   {
      this.numberOfFootstepsToConsider = new YoInteger(namePrefix + "AngularMomentumPlanMaxFootsteps", registry);
      this.swingLegMass = new YoDouble(namePrefix + "SwingFootMassForAngularMomentumEstimation", registry);
      this.supportLegMass = new YoDouble(namePrefix + "SupportFootMassForAngularMomentumEstimation", registry);
      this.comHeight = new YoDouble(namePrefix + "CoMHeightForAngularMomentumEstimation", registry);
      this.swingFootMaxHeight = new YoDouble(namePrefix + "SwingFootMaxHeightForAngularMomentumEstimation", registry);

      this.swingAngularMomentumTrajectories = new ArrayList<>(maxNumberOfFootstepsToConsider);
      this.transferAngularMomentumTrajectories = new ArrayList<>(maxNumberOfFootstepsToConsider + 1);
      this.upcomingCoPsInFootsteps = new ArrayList<>(maxNumberOfFootstepsToConsider + 2);

      ReferenceFrame[] referenceFrames = {worldFrame};
      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         SwingAngularMomentumTrajectory swingTrajectory = new SwingAngularMomentumTrajectory(namePrefix + "Footstep", i, registry, worldFrame, numberOfSwingSegments,
                                                                                             2 * maxNumberOfTrajectoryCoefficients);
         this.swingAngularMomentumTrajectories.add(swingTrajectory);
         TransferAngularMomentumTrajectory transferTrajectory = new TransferAngularMomentumTrajectory(namePrefix + "Footstep", i, registry, worldFrame, numberOfTransferSegments,
                                                                                                      2 * maxNumberOfTrajectoryCoefficients);
         this.transferAngularMomentumTrajectories.add(transferTrajectory);
         CoPPointsInFoot copLocations = new CoPPointsInFoot(i, referenceFrames, registry);
         upcomingCoPsInFootsteps.add(copLocations);
      }
      CoPPointsInFoot copLocations = new CoPPointsInFoot(maxNumberOfFootstepsToConsider, referenceFrames, registry);
      upcomingCoPsInFootsteps.add(copLocations);
      copLocations = new CoPPointsInFoot(maxNumberOfFootstepsToConsider + 1, referenceFrames, registry);
      upcomingCoPsInFootsteps.add(copLocations);
      TransferAngularMomentumTrajectory transferTrajectory = new TransferAngularMomentumTrajectory(namePrefix + "Footstep", maxNumberOfFootstepsToConsider, registry, worldFrame,
                                                                                                   numberOfTransferSegments, 2 * maxNumberOfTrajectoryCoefficients);
      this.transferAngularMomentumTrajectories.add(transferTrajectory);

      //this.footstepCoMTrajectory = new YoFrameTrajectory3D("EstFootstepCoMTrajectory", maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.segmentCoMTrajectory = new YoFrameTrajectory3D(namePrefix + "EstSegmentTrajectory", maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.segmentCoMVelocity = new YoFrameTrajectory3D(namePrefix + "EstSegmentCoMVelocity", maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.swingFootTrajectory = new YoFrameTrajectory3D(namePrefix + "EstSegmentSwingTrajectory", 2 * maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.swingFootVelocity = new YoFrameTrajectory3D(namePrefix + "EstSegmentSwingVelocity", maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.swingLiftTrajectory = new YoTrajectory(namePrefix + "SwingFootLiftTraj", maxNumberOfTrajectoryCoefficients, registry);
      this.supportFootTrajectory = new YoFrameTrajectory3D(namePrefix + "EstSegmentSupportTrajectory", 2 * maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.supportFootVelocity = new YoFrameTrajectory3D(namePrefix + "EstSegmentSupportVelocity", maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.estimatedAngularMomentumTrajectory = new YoFrameTrajectory3D("EstSegmentAngularMomenetum", 2 * maxNumberOfTrajectoryCoefficients, worldFrame,
                                                                        registry);
      this.previousEstimatedTransferTrajectory = new YoFrameTrajectory3D(namePrefix + "SaveEstAngMomTraj", 2 * maxNumberOfTrajectoryCoefficients, worldFrame, registry);

      // DEBUG 
      this.comTrajD = new YoFrameTrajectory3D("CoMDebugTraj", maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.swingTrajD = new YoFrameTrajectory3D("SwingDebugTraj", maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.anguMomTraj = new YoFrameVector("AngMomViz", worldFrame, registry);
      this.comPos = new YoFramePoint("CoMViz", "", worldFrame, registry);
      this.swingFootTraj = new YoFramePoint("SwFViz", worldFrame, registry);
      parentRegistry.addChild(registry);
   }

   public void initializeParameters(AngularMomentumEstimationParameters angularMomentumParameters)
   {
      this.numberOfFootstepsToConsider.set(angularMomentumParameters.getNumberOfFootstepsToConsider());
      this.trajectoryPointStart = angularMomentumParameters.getInitialCoPPointName();
      this.trajectoryInitialDepartureReference = angularMomentumParameters.getInitialDepartureReferenceName();
      this.trajectoryPointEnd = angularMomentumParameters.getEndCoPName();
      this.trajectoryFinalApproachReference = angularMomentumParameters.getFinalApproachReferenceName();
      this.entryCoP = angularMomentumParameters.getEntryCoPName();
      this.exitCoP = angularMomentumParameters.getExitCoPName();
      this.endCoP = angularMomentumParameters.getEndCoPName();
      this.transferCoPList = angularMomentumParameters.getCoPPlannerParameters().getTransferCoPPointsToPlan();
      this.swingLegMass.set(angularMomentumParameters.getSwingLegMass());
      this.supportLegMass.set(angularMomentumParameters.getSupportLegMass());
      this.comHeight.set(angularMomentumParameters.getCoMHeight());
      this.swingFootMaxHeight.set(angularMomentumParameters.getSwingFootMaxLift());
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
      }
      for (int i = 0; i < upcomingCoPsInFootsteps.size(); i++)
         upcomingCoPsInFootsteps.get(i).reset();
   }

   public void addFootstepCoPsToPlan(List<CoPPointsInFoot> copLocations)
   {
      for (int i = 0; i < copLocations.size(); i++)
      {
         upcomingCoPsInFootsteps.get(i).setIncludingFrame(copLocations.get(i));
         //PrintTools.debug(i + " " + upcomingCoPsInFootsteps.get(i).toString());
      }
   }

   @Override
   public void update(double currentTime)
   {
      if (activeTrajectory != null)
         activeTrajectory.update(currentTime - initialTime, desiredAngularMomentum, desiredTorque, desiredRotatum);
   }

   @Override
   public void getDesiredAngularMomentum(FrameVector desiredAngMomToPack)
   {
      desiredAngMomToPack.setIncludingFrame(desiredAngularMomentum);
   }

   @Override
   public void getDesiredAngularMomentum(FrameVector desiredAngMomToPack, FrameVector desiredTorqueToPack)
   {
      desiredAngMomToPack.setIncludingFrame(desiredAngularMomentum);
      desiredTorqueToPack.setIncludingFrame(desiredTorque);
   }

   public void getDesiredAngularMomentum(FrameVector desiredAngMomToPack, FrameVector desiredTorqueToPack, FrameVector desiredRotatumToPack)
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
   }

   @Override
   public void initializeForSwing(double currentTime)
   {
      initialTime = currentTime;
      activeTrajectory = swingAngularMomentumTrajectories.get(0);
   }
   private double additionalInitialSwingTime = 0.0;
   @Override
   public void computeReferenceAngularMomentumStartingFromDoubleSupport(boolean atAStop)
   {
//      PrintTools.debug("Double Support");
      firstSwing = firstCoM = true;
      swingStart = false;
      footstepIndex = 0;
      if (atAStop)
      {
         upcomingCoPsInFootsteps.get(footstepIndex).get(0).getPosition(previousFinalReferenceCoP);
         previousFirstTransferEndTime = 0.0;
         updateCurrentSegmentTimes(footstepIndex);
         currentSecondTransferSegmentDuration += upcomingCoPsInFootsteps.get(footstepIndex + 1).get(0).getTime();
         currentFootstepTime += upcomingCoPsInFootsteps.get(footstepIndex + 1).get(0).getTime();
         additionalInitialSwingTime = upcomingCoPsInFootsteps.get(footstepIndex + 1).get(0).getTime();
         computeAngularMomentumApproximationForFootstep();
         footstepIndex++;
      }
      else
      {
         additionalInitialSwingTime = 0.0;
         upcomingCoPsInFootsteps.get(footstepIndex).get(CoPPlanningTools.getCoPPointIndex(upcomingCoPsInFootsteps.get(footstepIndex).getCoPPointList(), trajectoryFinalApproachReference)).getPosition(previousFinalReferenceCoP);
         // Use the previously planned trajectory from the single support
         transferAngularMomentumTrajectories.get(footstepIndex).set(previousEstimatedTransferTrajectory);
         previousFirstTransferEndTime = previousEstimatedTransferTrajectory.getFinalTime();
      }
      computeAngularMomentumApproximationForUpcomingFootsteps();
   }

   @Override
   public void computeReferenceAngularMomentumStartingFromSingleSupport()
   {
      firstSwing = firstCoM = true;
      swingStart = true;
      footstepIndex = 0;
      previousFirstTransferEndTime = 0.0;
      updateCurrentSegmentTimes(footstepIndex);
      currentSecondTransferSegmentDuration += additionalInitialSwingTime;
      currentFootstepTime += additionalInitialSwingTime;
      computeAngularMomentumApproximationForFootstep();
      previousEstimatedTransferTrajectory.set(transferAngularMomentumTrajectories.get(footstepIndex + 1).getSegments().get(0));
      footstepIndex++;
      computeAngularMomentumApproximationForUpcomingFootsteps();
   }

   private void computeAngularMomentumApproximationForUpcomingFootsteps()
   {
      for (; footstepIndex + 2 < upcomingCoPsInFootsteps.size() && (upcomingCoPsInFootsteps.get(footstepIndex + 1).getCoPPointList().size() > 1); footstepIndex++)
      {
         updateCurrentSegmentTimes(footstepIndex);
         //setCoMTrajectoryForFootstep(footstepIndex);
         computeAngularMomentumApproximationForFootstep();
      }
   }

   // This function assumes that all setup for the footstep has been carried out already 
   private void computeAngularMomentumApproximationForFootstep()
   {
      computeAngularMomentumApproximationForFootstep(TrajectorySegment.END_TO_ENTRY);
   }

   private void computeAngularMomentumApproximationForFootstep(TrajectorySegment startFromSegment)
   {
      switch (startFromSegment)
      {
      case END_TO_ENTRY:
         computeAngularMomentumForSecondTransferSegment();
      case ENTRY_TO_EXIT:
         computeAngularMomentumForSwing();
      default:
         computeAngularMomentumForFirstTransfer();
      }
      previousFirstTransferEndTime = currentFirstTransferSegmentDuration;
   }

   private void computeAngularMomentumForSecondTransferSegment()
   {
      setCoMTrajectoryForSecondTransfer(previousFirstTransferEndTime);
      setSwingFootTrajectoryForSecondTransfer(footstepIndex, previousFirstTransferEndTime);
      setSupportFootTrajectoryForSecondTransfer(footstepIndex, previousFirstTransferEndTime);
      calculateAngularMomentumTrajectory();
      transferAngularMomentumTrajectories.get(footstepIndex).set(estimatedAngularMomentumTrajectory);
   }

   private void computeAngularMomentumForSwing()
   {
      setCoMTrajectoryForSwing();
      setSwingFootTrajectoryForSwing(footstepIndex);
      setSupportFootTrajectoryForSwing(footstepIndex);
      calculateAngularMomentumTrajectory();
      swingAngularMomentumTrajectories.get(footstepIndex).set(estimatedAngularMomentumTrajectory);
   }

   private void computeAngularMomentumForFirstTransfer()
   {
      setCoMTrajectoryForFirstTransfer();
      setSwingFootTrajectoryForFirstTransfer(footstepIndex);
      setSupportFootTrajectoryForFirstTransfer(footstepIndex);
      calculateAngularMomentumTrajectory();
      transferAngularMomentumTrajectories.get(footstepIndex + 1).set(estimatedAngularMomentumTrajectory);
   }

   private void calculateAngularMomentumTrajectory()
   {
      swingFootTrajectory.subtract(segmentCoMTrajectory);
      swingFootVelocity.subtract(segmentCoMVelocity);
      supportFootTrajectory.subtract(segmentCoMTrajectory);
      supportFootVelocity.subtract(segmentCoMVelocity);

      swingFootTrajectory.crossProduct(swingFootVelocity);
      swingFootTrajectory.scale(swingLegMass.getDoubleValue());
      supportFootTrajectory.crossProduct(supportFootVelocity);
      supportFootTrajectory.scale(supportLegMass.getDoubleValue());
      estimatedAngularMomentumTrajectory.add(supportFootTrajectory, swingFootTrajectory);
   }

   private void setSupportFootTrajectoryForSwing(int footstepIndex)
   {
      setSupportFootTrajectory(footstepIndex, 0.0, currentSwingSegmentDuration);
   }

   private void setSupportFootTrajectoryForFirstTransfer(int footstepIndex)
   {
      setSupportFootTrajectory(footstepIndex, 0.0, currentFirstTransferSegmentDuration);
   }

   private void setSupportFootTrajectoryForSecondTransfer(int footstepIndex, double segmentStartTime)
   {
      setSupportFootTrajectory(footstepIndex, segmentStartTime, segmentStartTime + currentSecondTransferSegmentDuration);
   }

   private void setSupportFootTrajectory(int footstepIndex, double startTime, double endTime)
   {
      upcomingCoPsInFootsteps.get(footstepIndex + 1).getSupportFootLocation(tempFramePoint1);
      supportFootTrajectory.setConstant(startTime, endTime, tempFramePoint1);
      supportFootTrajectory.getDerivative(supportFootVelocity);
   }

   private void setSwingFootTrajectoryForSwing(int footstepIndex)
   {
      upcomingCoPsInFootsteps.get(footstepIndex).getSupportFootLocation(tempFramePoint1);
      upcomingCoPsInFootsteps.get(footstepIndex + 1).getSwingFootLocation(tempFramePoint2);
      swingFootTrajectory.setPenticWithZeroTerminalAcceleration(0.0, currentSwingSegmentDuration, tempFramePoint1, zeroVector, tempFramePoint2, zeroVector);
      updateSwingLiftTrajectory();
      //swingFootTrajectory.getYoTrajectoryZ().add(swingLiftTrajectory);
      swingFootTrajectory.getDerivative(swingFootVelocity);

      if (firstSwing && swingStart)
      {
         swingTrajD.set(swingFootTrajectory);
         firstSwing = false;
      }
   }

   private void updateSwingLiftTrajectory()
   {
      swingLiftTrajectory.setQuadraticUsingIntermediatePoint(0, currentSwingSegmentDuration / 2.0, currentSwingSegmentDuration, 0.0,
                                                             swingFootMaxHeight.getDoubleValue(), 0.0);
   }

   // First transfer - exitCoP to endCoP
   // Second transfer - endCoP to entry CoP
   private void setSwingFootTrajectoryForFirstTransfer(int footstepIndex)
   {
      upcomingCoPsInFootsteps.get(footstepIndex + 1).getSwingFootLocation(tempFramePoint1);
      swingFootTrajectory.setConstant(0.0, currentFirstTransferSegmentDuration, tempFramePoint1);
      swingFootTrajectory.getDerivative(swingFootVelocity);

      if (firstSwing && !swingStart)
      {
         swingTrajD.set(swingFootTrajectory);
         firstSwing = false;
      }
   }

   private void setSwingFootTrajectoryForSecondTransfer(int footstepIndex, double segmentStartTime)
   {
      upcomingCoPsInFootsteps.get(footstepIndex).getSupportFootLocation(tempFramePoint1);
      swingFootTrajectory.setConstant(segmentStartTime, segmentStartTime + currentSecondTransferSegmentDuration, tempFramePoint1);
      swingFootTrajectory.getDerivative(swingFootVelocity);

      if (firstSwing && !swingStart)
      {
         swingTrajD.set(swingFootTrajectory);
         firstSwing = false;
      }
   }

   private void setCoMTrajectoryForSwing()
   {
      getCoMEstimationWaypoints();
      //segmentCoMTrajectory.setCubicBezier(-currentSecondTransferSegmentDuration, -currentSecondTransferSegmentDuration + currentFootstepTime, tempFramePoint1,
      //                              tempFramePoint2, tempFramePoint3, tempFramePoint4);
      segmentCoMTrajectory.setPenticWithZeroTerminalAcceleration(-currentSecondTransferSegmentDuration, -currentSecondTransferSegmentDuration + currentFootstepTime, tempFramePoint1,
                                    tempFrameVector1, tempFramePoint2, tempFrameVector2);
      segmentCoMTrajectory.setTime(0.0, currentSwingSegmentDuration);
      segmentCoMTrajectory.getDerivative(segmentCoMVelocity);
     
      if (firstCoM && swingStart)
      {
         comTrajD.set(segmentCoMTrajectory);
         firstCoM = false;
      }
   }

   private void setCoMTrajectoryForFirstTransfer()
   {
      getCoMEstimationWaypoints();
      //segmentCoMTrajectory.setCubicBezier(-(currentSecondTransferSegmentDuration + currentSwingSegmentDuration), currentFirstTransferSegmentDuration, tempFramePoint1,
      //                              tempFramePoint2, tempFramePoint3, tempFramePoint4);
      segmentCoMTrajectory.setPenticWithZeroTerminalAcceleration(-(currentSecondTransferSegmentDuration + currentSwingSegmentDuration), currentFirstTransferSegmentDuration, tempFramePoint1,
                                                                        tempFrameVector1, tempFramePoint2, tempFrameVector2);                                          
      segmentCoMTrajectory.setTime(0.0, currentFirstTransferSegmentDuration);
      segmentCoMTrajectory.getDerivative(segmentCoMVelocity);
      
      if (firstCoM && !swingStart)
      {
         comTrajD.set(segmentCoMTrajectory);
         firstCoM = false;
      }
   }

   private void setCoMTrajectoryForSecondTransfer(double startTime)
   {
      getCoMEstimationWaypoints();
      //segmentCoMTrajectory.setCubicBezier(startTime, startTime + currentFootstepTime, tempFramePoint1, tempFramePoint2, tempFramePoint3, tempFramePoint4);
      segmentCoMTrajectory.setPenticWithZeroTerminalAcceleration(startTime, startTime + currentFootstepTime, tempFramePoint1, tempFrameVector1, tempFramePoint2, tempFrameVector2);
      segmentCoMTrajectory.setTime(startTime, startTime + currentSecondTransferSegmentDuration);
      segmentCoMTrajectory.getDerivative(segmentCoMVelocity);
      
      if (firstCoM && !swingStart)
      {
         comTrajD.set(segmentCoMTrajectory);
         firstCoM = false;
      }
   }

   private void getCoMEstimationWaypoints()
   {
      upcomingCoPsInFootsteps.get(footstepIndex).getSwingFootLocation(tempFramePoint3);
      upcomingCoPsInFootsteps.get(footstepIndex).getSupportFootLocation(tempFramePoint4);
      tempFramePoint1.interpolate(tempFramePoint3, tempFramePoint4, 0.5);
      upcomingCoPsInFootsteps.get(footstepIndex + 1).getSwingFootLocation(tempFramePoint3);
      upcomingCoPsInFootsteps.get(footstepIndex + 1).getSupportFootLocation(tempFramePoint4);
      tempFramePoint2.interpolate(tempFramePoint3, tempFramePoint4, 0.5);
      
      if(footstepIndex == 0)
      {
         upcomingCoPsInFootsteps.get(footstepIndex + 1).get(CoPPlanningTools.getCoPPointIndex(upcomingCoPsInFootsteps.get(footstepIndex + 1).getCoPPointList(), trajectoryInitialDepartureReference)).getPosition(tempFramePoint3);
         tempFrameVector1.sub(tempFramePoint3, previousFinalReferenceCoP);
      }
      else
      {
         upcomingCoPsInFootsteps.get(footstepIndex + 1).get(CoPPlanningTools.getCoPPointIndex(upcomingCoPsInFootsteps.get(footstepIndex + 1).getCoPPointList(), trajectoryInitialDepartureReference)).getPosition(tempFramePoint3);
         upcomingCoPsInFootsteps.get(footstepIndex).get(CoPPlanningTools.getCoPPointIndex(upcomingCoPsInFootsteps.get(footstepIndex).getCoPPointList(), trajectoryFinalApproachReference)).getPosition(tempFramePoint4);
         tempFrameVector1.sub(tempFramePoint3, tempFramePoint4);
      }
      
      if(footstepIndex + 3 < upcomingCoPsInFootsteps.size() && !upcomingCoPsInFootsteps.get(footstepIndex + 3).getCoPPointList().isEmpty())
      {
         upcomingCoPsInFootsteps.get(footstepIndex + 1).get(CoPPlanningTools.getCoPPointIndex(upcomingCoPsInFootsteps.get(footstepIndex + 1).getCoPPointList(), trajectoryFinalApproachReference)).getPosition(tempFramePoint3);
         upcomingCoPsInFootsteps.get(footstepIndex + 2).get(CoPPlanningTools.getCoPPointIndex(upcomingCoPsInFootsteps.get(footstepIndex + 2).getCoPPointList(), trajectoryInitialDepartureReference)).getPosition(tempFramePoint4);
         tempFrameVector2.sub(tempFramePoint4, tempFramePoint3);
      }
      else
      {         
         upcomingCoPsInFootsteps.get(footstepIndex + 1).get(CoPPlanningTools.getCoPPointIndex(upcomingCoPsInFootsteps.get(footstepIndex + 1).getCoPPointList(), trajectoryFinalApproachReference)).getPosition(tempFramePoint3);
         tempFrameVector2.sub(tempFramePoint2, tempFramePoint3);
      }
      tempFramePoint1.add(0, 0, comHeight.getDoubleValue());
      tempFramePoint2.add(0, 0, comHeight.getDoubleValue());
   }

   private void updateCurrentSegmentTimes(int footstepIndex)
   {
      this.currentFootstepTime = 0.0;
      currentCoPPlanReference = upcomingCoPsInFootsteps.get(footstepIndex + 1);
      currentCoPListReference = currentCoPPlanReference.getCoPPointList();
      if (currentCoPListReference.isEmpty())
         return;
      currentSecondTransferSegmentDuration = 0.0;
      for (tempInt1 = CoPPlanningTools.getCoPPointIndex(currentCoPListReference, endCoP) + 1; currentCoPListReference.get(tempInt1) != entryCoP; tempInt1++)
         currentSecondTransferSegmentDuration += currentCoPPlanReference.get(tempInt1).getTime();
      currentSecondTransferSegmentDuration += currentCoPPlanReference.get(tempInt1).getTime();
      tempInt1++;
      currentSwingSegmentDuration = 0.0;
      for (; tempInt1 < currentCoPListReference.size(); tempInt1++)
         currentSwingSegmentDuration += currentCoPPlanReference.get(tempInt1).getTime();
      currentFirstTransferSegmentDuration = 0.0;
      currentCoPPlanReference = upcomingCoPsInFootsteps.get(footstepIndex + 2);
      currentCoPListReference = currentCoPPlanReference.getCoPPointList();
      for (tempInt1 = 0; currentCoPListReference.get(tempInt1) != endCoP; tempInt1++)
         currentFirstTransferSegmentDuration += currentCoPPlanReference.get(tempInt1).getTime();
      currentFirstTransferSegmentDuration += currentCoPPlanReference.get(tempInt1).getTime();
      currentFootstepTime = currentSecondTransferSegmentDuration + currentSwingSegmentDuration + currentFirstTransferSegmentDuration;
   }

   public void getPredictedCenterOfMassPosition(FramePoint pointToPack, double time)
   {
      comTrajD.compute(time - initialTime);
      comTrajD.getFramePosition(pointToPack);
      comPos.set(pointToPack);
   }

   public void getPredictedCenterOfMassPosition(YoFramePoint pointToPack, double time)
   {
      comTrajD.compute(time - initialTime);
      FramePoint comPosition = comTrajD.getFramePosition();
      pointToPack.set(comPosition);
      comPos.set(comPosition);
   }

   public void getPredictedSwingFootPosition(FramePoint pointToPack, double time)
   {
      if (!swingStart)
         transferAngularMomentumTrajectories.get(0).update(time - initialTime, tempFramePoint1);
      else
         swingAngularMomentumTrajectories.get(0).update(time - initialTime, tempFramePoint1);
      anguMomTraj.set(tempFramePoint1);

      swingTrajD.compute(time - initialTime);
      swingTrajD.getFramePosition(pointToPack);
      swingFootTraj.set(pointToPack);
   }

   public void getPredictedSwingFootPosition(YoFramePoint pointToPack, double time)
   {
      if (!swingStart)
         transferAngularMomentumTrajectories.get(0).update(time - initialTime, tempFramePoint1);
      else
         swingAngularMomentumTrajectories.get(0).update(time - initialTime, tempFramePoint1);
      anguMomTraj.set(tempFramePoint1);

      swingTrajD.compute(time - initialTime);

      FramePoint predictedSwingFootPosition = swingTrajD.getFramePosition();
      pointToPack.set(predictedSwingFootPosition);
      swingFootTraj.set(predictedSwingFootPosition);
   }

   @Override
   public List<? extends AngularMomentumTrajectory> getTransferAngularMomentumTrajectories()
   {
      return transferAngularMomentumTrajectories;
   }

   @Override
   public List<? extends AngularMomentumTrajectory> getSwingAngularMomentumTrajectories()
   {
      return swingAngularMomentumTrajectories;
   }
}
