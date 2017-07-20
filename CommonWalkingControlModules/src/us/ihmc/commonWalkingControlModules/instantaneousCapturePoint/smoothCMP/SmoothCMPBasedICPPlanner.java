package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPTrajectoryPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.AbstractICPPlanner;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SmoothCMPBasedICPPlanner extends AbstractICPPlanner
{
   private static final boolean VISUALIZE = true;

   private final ReferenceCoPTrajectoryGenerator referenceCoPGenerator;
   private final ReferenceCMPTrajectoryGenerator referenceCMPGenerator;
   private final ReferenceICPTrajectoryGenerator referenceICPGenerator;

   private final List<YoDouble> swingDurationShiftFractions = new ArrayList<>();

   public SmoothCMPBasedICPPlanner(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                   int maxNumberOfFootstepsToConsider, int numberOfPointsPerFoot, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(bipedSupportPolygons, maxNumberOfFootstepsToConsider);

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         YoDouble swingDurationShiftFraction = new YoDouble("swingDurationShiftFraction" + i, registry);
         swingDurationShiftFractions.add(swingDurationShiftFraction);
      }

      referenceCoPGenerator = new ReferenceCoPTrajectoryGenerator(namePrefix, numberOfPointsPerFoot, maxNumberOfFootstepsToConsider, bipedSupportPolygons, contactableFeet,
                                                                  numberFootstepsToConsider, swingDurations, transferDurations, swingDurationAlphas,
                                                                  swingDurationShiftFractions, transferDurationAlphas,
                                                                  registry);

      referenceCMPGenerator = new ReferenceCMPTrajectoryGenerator(namePrefix, maxNumberOfFootstepsToConsider, numberFootstepsToConsider, swingDurations, transferDurations,
                                                                  swingDurationAlphas, transferDurationAlphas, registry);

      referenceICPGenerator = new ReferenceICPTrajectoryGenerator(namePrefix, omega0, numberFootstepsToConsider, isStanding, useDecoupled, worldFrame,
                                                                  registry);

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry != null)
      {
         setupVisualizers(yoGraphicsListRegistry);
      }
   }

   public void initializeParameters(ICPPlannerParameters icpPlannerParameters)
   {
      super.initializeParameters((ICPTrajectoryPlannerParameters) icpPlannerParameters);

      if (icpPlannerParameters instanceof SmoothCMPPlannerParameters)
      {
         numberFootstepsToConsider.set(icpPlannerParameters.getNumberOfFootstepsToConsider());

         referenceCoPGenerator.initializeParameters((SmoothCMPPlannerParameters) icpPlannerParameters);

         for (int i = 0; i < numberFootstepsToConsider.getIntegerValue(); i++)
         {
            swingDurationShiftFractions.get(i).set(((SmoothCMPPlannerParameters) icpPlannerParameters).getSwingDurationShiftFraction());
         }
      }
      else
      {
         throw new RuntimeException("Tried to submit the wrong type of parameters.");
      }
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      referenceCoPGenerator.createVisualizerForConstantCoPs(yoGraphicsList, artifactList);

      artifactList.setVisible(VISUALIZE);
      yoGraphicsList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   @Override
   /** {@inheritDoc} */
   public void clearPlan()
   {
      referenceCoPGenerator.clear();
      referenceCMPGenerator.reset();
      referenceICPGenerator.reset();

      for (int i = 0; i < swingDurations.size(); i++)
      {
         swingDurations.get(i).setToNaN();
         transferDurations.get(i).setToNaN();
         swingDurationAlphas.get(i).setToNaN();
         transferDurationAlphas.get(i).setToNaN();
      }
   }

   @Override
   /** {@inheritDoc} */
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      referenceCoPGenerator.addFootstepToPlan(footstep, timing);
   }

   @Override
   /** {@inheritDoc} */
   public void initializeForStanding(double initialTime)
   {
      clearPlan();

      this.initialTime.set(initialTime);

      isStanding.set(true);
      isDoubleSupport.set(true);

      
      transferDurations.get(0).set(finalTransferDuration.getDoubleValue());
      transferDurationAlphas.get(0).set(finalTransferDurationAlpha.getDoubleValue());
      
      updateTransferPlan();    
   }

   @Override
   /** {@inheritDoc} */
   public void initializeForTransfer(double initialTime)
   {
      this.initialTime.set(initialTime);

      isStanding.set(false);
      isDoubleSupport.set(true);      
      
      int numberOfFootstepRegistered = getNumberOfFootstepsRegistered();
      if (numberOfFootstepRegistered < numberOfFootstepsToConsider.getIntegerValue())
      {
         transferDurations.get(numberOfFootstepRegistered).set(finalTransferDuration.getDoubleValue());
         transferDurationAlphas.get(numberOfFootstepRegistered).set(finalTransferDurationAlpha.getDoubleValue());
      }

      updateTransferPlan();
   }

   @Override
   /** {@inheritDoc} */
   public void computeFinalCoMPositionInTransfer()
   {
      throw new RuntimeException("to implement"); //TODOLater
   }

   @Override
   /** {@inheritDoc} */
   public void initializeForSingleSupport(double initialTime)
   {      
      this.initialTime.set(initialTime);

      isStanding.set(false);
      isDoubleSupport.set(false);
      
      isInitialTransfer.set(false);
      isHoldingPosition.set(false);

      
      int numberOfFootstepRegistered = getNumberOfFootstepsRegistered();
      if (numberOfFootstepRegistered < numberOfFootstepsToConsider.getIntegerValue())
      {
         transferDurations.get(numberOfFootstepRegistered).set(finalTransferDuration.getDoubleValue());
         transferDurationAlphas.get(numberOfFootstepRegistered).set(finalTransferDurationAlpha.getDoubleValue());
      }

//      yoSingleSupportInitialCoM.set(desiredCoMPosition);
//      desiredCoMPosition.getFrameTuple2d(singleSupportInitialCoM);
      
      updateSingleSupportPlan();
   }

   @Override
   /** {@inheritDoc} */
   public void computeFinalCoMPositionInSwing()
   {
      throw new RuntimeException("to implement"); //TODOLater
   }

   @Override
   /** {@inheritDoc} */
   protected void updateTransferPlan()
   {
      RobotSide transferToSide = this.transferToSide.getEnumValue();
      
      if (transferToSide == null)
         transferToSide = RobotSide.LEFT;

      if (isStanding.getBooleanValue())
         referenceCoPGenerator.computeReferenceCoPsStartingFromDoubleSupport(true, transferToSide);
      else
         referenceCoPGenerator.computeReferenceCoPsStartingFromDoubleSupport(false, transferToSide);
      referenceCMPGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());
      referenceICPGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());
      
      referenceCoPGenerator.initializeForTransfer(this.initialTime.getDoubleValue());
      referenceCMPGenerator.initializeForTransfer(this.initialTime.getDoubleValue(), referenceCoPGenerator.getTransferCoPTrajectories(), referenceCoPGenerator.getSwingCoPTrajectories());
      referenceICPGenerator.initializeForTransfer(this.initialTime.getDoubleValue(), referenceCMPGenerator.getTransferCMPTrajectories(), referenceCMPGenerator.getSwingCMPTrajectories());
   }

   @Override
   /** {@inheritDoc} */
   protected void updateSingleSupportPlan()
   {
      RobotSide supportSide = this.supportSide.getEnumValue();
      
      referenceCoPGenerator.computeReferenceCoPsStartingFromSingleSupport(supportSide);
      referenceCMPGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());
      referenceICPGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());
      
      referenceCoPGenerator.initializeForSwing(this.initialTime.getDoubleValue());
      referenceCMPGenerator.initializeForSwing(this.initialTime.getDoubleValue(), referenceCoPGenerator.getTransferCoPTrajectories(), referenceCoPGenerator.getSwingCoPTrajectories());
      referenceICPGenerator.initializeForSwing(this.initialTime.getDoubleValue(), referenceCMPGenerator.getTransferCMPTrajectories(), referenceCMPGenerator.getSwingCMPTrajectories());
   }

   @Override
   /** {@inheritDoc} */
   public void compute(double time)
   {
      referenceCoPGenerator.update(time);
      referenceCMPGenerator.update(time);
      referenceICPGenerator.compute(time);

      referenceCoPGenerator.getDesiredCenterOfPressure(desiredCoPPosition, desiredCoPVelocity);
      referenceCMPGenerator.getLinearData(desiredCMPPosition, desiredCMPVelocity);
      referenceICPGenerator.getLinearData(desiredICPPosition, desiredICPVelocity, desiredICPAcceleration);      
   }
   
   /** {@inheritDoc} */
   public void updateListeners()
   {
      referenceCoPGenerator.updateListeners();     
   }
   
   /** {@inheritDoc} */
   public List<CoPPointsInFoot> getCoPWaypoints()
   {
      return referenceCoPGenerator.getWaypoints();     
   }
   
   @Override
   /** {@inheritDoc} */
   public void getFinalDesiredCapturePointPosition(FramePoint finalDesiredCapturePointPositionToPack)
   {
      throw new RuntimeException("to implement"); //TODO
   }

   @Override
   /** {@inheritDoc} */
   public void getFinalDesiredCapturePointPosition(YoFramePoint2d finalDesiredCapturePointPositionToPack)
   {
      throw new RuntimeException("to implement"); //TODO
   }

   @Override
   /** {@inheritDoc} */
   public void getFinalDesiredCenterOfMassPosition(FramePoint2d finalDesiredCenterOfMassPositionToPack)
   {
      throw new RuntimeException("to implement"); //TODOLater
   }

   @Override
   /** {@inheritDoc} */
   public void getNextExitCMP(FramePoint entryCMPToPack)
   {
      List<CoPPointsInFoot> plannedCoPWaypoints = referenceCoPGenerator.getWaypoints();
      plannedCoPWaypoints.get(1).get(this.exitCoPName).getPosition(entryCMPToPack);
   }

   @Override
   /** {@inheritDoc} */
   public boolean isOnExitCMP()
   {
      throw new RuntimeException("to implement"); //TODO
   }

   @Override
   /** {@inheritDoc} */
   public int getNumberOfFootstepsToConsider()
   {
      return numberOfFootstepsToConsider.getIntegerValue();
   }

   @Override
   /** {@inheritDoc} */
   public int getNumberOfFootstepsRegistered()
   {
      return referenceCoPGenerator.getNumberOfFootstepsRegistered();
   }
}
