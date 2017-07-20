package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.AngularMomentumTrajectoryInterface;
import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class ReferenceCMPTrajectoryGenerator
{
   private final List<YoDouble> swingDurations;
   private final List<YoDouble> swingSplitFractions;

   private final List<YoDouble> transferDurations;
   private final List<YoDouble> transferSplitFractions;

   private final List<CMPTrajectory> transferCMPTrajectories = new ArrayList<>();
   private final List<CMPTrajectory> swingCMPTrajectories = new ArrayList<>();

   private final YoInteger numberOfFootstepsToConsider;

   private double initialTime;
   private int numberOfRegisteredSteps;
   private CMPTrajectory activeTrajectory;

   private final FramePoint desiredCMP = new FramePoint();
   private final FrameVector desiredCMPVelocity = new FrameVector();
   private CMPTrajectory cmpTrajectoryReference;
   private CoPTrajectoryInterface copTrajectoryReference;
   private AngularMomentumTrajectoryInterface angularMomentumTrajectory;
   
   public ReferenceCMPTrajectoryGenerator(String namePrefix, YoInteger numberOfFootstepsToConsider, List<YoDouble> swingDurations,
                                          List<YoDouble> transferDurations, List<YoDouble> swingSplitFractions, List<YoDouble> transferSplitFractions,
                                          YoVariableRegistry registry)
   {
      this.swingDurations = swingDurations;
      this.transferDurations = transferDurations;
      this.swingSplitFractions = swingSplitFractions;
      this.transferSplitFractions = transferSplitFractions;

      this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;

      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         CMPTrajectory transferCMPTrajectory = new CMPTrajectory(namePrefix + "Transfer" + i, registry);
         CMPTrajectory swingCMPTrajectory = new CMPTrajectory(namePrefix + "Swing" + i, registry);
         transferCMPTrajectories.add(transferCMPTrajectory);
         swingCMPTrajectories.add(swingCMPTrajectory);
      }
   }

   public void reset()
   {
      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         transferCMPTrajectories.get(i).reset();
         swingCMPTrajectories.get(i).reset();
      }
      transferCMPTrajectories.get(numberOfFootstepsToConsider.getIntegerValue()).reset();
      activeTrajectory = null;
   }

   public void update(double currentTime)
   {
      double timeInState = currentTime - initialTime;

      if (activeTrajectory != null)
         activeTrajectory.update(timeInState, desiredCMP, desiredCMPVelocity);
   }

   public void getPosition(FramePoint desiredCMPToPack)
   {
      desiredCMPToPack.setIncludingFrame(desiredCMP);
   }

   public void getPosition(YoFramePoint desiredCMPToPack)
   {
      desiredCMPToPack.set(desiredCMP);
   }

   public void getVelocity(FrameVector desiredCMPVelocityToPack)
   {
      desiredCMPVelocityToPack.setIncludingFrame(desiredCMPVelocity);
   }

   public void getVelocity(YoFrameVector desiredCMPVelocityToPack)
   {
      desiredCMPVelocityToPack.set(desiredCMPVelocity);
   }

   public void getLinearData(FramePoint positionToPack, FrameVector velocityToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
   }

   public void getLinearData(YoFramePoint positionToPack, YoFrameVector velocityToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
   }

   public List<CMPTrajectory> getTransferCMPTrajectories()
   {
      return transferCMPTrajectories;
   }

   public List<CMPTrajectory> getSwingCMPTrajectories()
   {
      return swingCMPTrajectories;
   }

   public void setNumberOfRegisteredSteps(int numberOfRegisteredSteps)
   {
      this.numberOfRegisteredSteps = numberOfRegisteredSteps;
   }

   public void initializeForTransfer(double currentTime, List<? extends CoPTrajectoryInterface> transferCoPTrajectories,
                                     List<? extends CoPTrajectoryInterface> swingCoPTrajectories,
                                     List<? extends AngularMomentumTrajectoryInterface> transferAngularMomentumTrajectories,
                                     List<? extends AngularMomentumTrajectoryInterface> swingAngulatMomentumTrajectories)
   {
      setCMPTrajectories(transferCoPTrajectories, swingCoPTrajectories, transferAngularMomentumTrajectories, swingAngulatMomentumTrajectories);
      activeTrajectory = transferCMPTrajectories.get(0);
   }

   public void initializeForSwing(double currentTime, List<? extends CoPTrajectoryInterface> transferCoPTrajectories,
                                  List<? extends CoPTrajectoryInterface> swingCoPTrajectories,
                                  List<? extends AngularMomentumTrajectoryInterface> transferAngularMomentumTrajectories,
                                  List<? extends AngularMomentumTrajectoryInterface> swingAngulatMomentumTrajectories)
   {
      setCMPTrajectories(transferCoPTrajectories, swingCoPTrajectories, transferAngularMomentumTrajectories, swingAngulatMomentumTrajectories);
      activeTrajectory = swingCMPTrajectories.get(0);
   }

   private void setCMPTrajectories(List<? extends CoPTrajectoryInterface> transferCoPTrajectories, List<? extends CoPTrajectoryInterface> swingCoPTrajectories,
                                   List<? extends AngularMomentumTrajectoryInterface> transferAngularMomentumTrajectories,
                                   List<? extends AngularMomentumTrajectoryInterface> swingAngulartMomentumTrajectories)
   {
      if (transferAngularMomentumTrajectories == null || swingAngulartMomentumTrajectories == null)
      {
         copyCoPTrajectoriesToCMPTrajectories(transferCoPTrajectories, swingCoPTrajectories, transferAngularMomentumTrajectories,
                                              swingAngulartMomentumTrajectories);
         return;
      }
      
      int numberOfFootstepsToCopy = Math.min(numberOfFootstepsToConsider.getIntegerValue(), numberOfRegisteredSteps);
      for (int i = 0; i < numberOfFootstepsToCopy; i++)
      {
         cmpTrajectoryReference = transferCMPTrajectories.get(i);
         copTrajectoryReference = transferCoPTrajectories.get(i);
         angularMomentumTrajectory = transferAngularMomentumTrajectories.get(i);
         
         
      }
   }

   private void copyCoPTrajectoriesToCMPTrajectories(List<? extends CoPTrajectoryInterface> transferCoPTrajectories,
                                                     List<? extends CoPTrajectoryInterface> swingCoPTrajectories,
                                                     List<? extends AngularMomentumTrajectoryInterface> transferAngularMomentumTrajectories,
                                                     List<? extends AngularMomentumTrajectoryInterface> swingAngulartMomentumTrajectories)
   {
      int numberOfFootstepsToCopy = Math.min(numberOfFootstepsToConsider.getIntegerValue(), numberOfRegisteredSteps);
      for (int i = 0; i < numberOfFootstepsToCopy; i++)
      {
         cmpTrajectoryReference = transferCMPTrajectories.get(i);
         copTrajectoryReference = transferCoPTrajectories.get(i);
         for (int j = 0; j < copTrajectoryReference.getNumberOfSegments(); j++)
            cmpTrajectoryReference.getPolynomials().set(j, copTrajectoryReference.getPolynomials().get(j));
         cmpTrajectoryReference = swingCMPTrajectories.get(i);
         copTrajectoryReference = swingCoPTrajectories.get(i);
         for (int j = 0; j < copTrajectoryReference.getNumberOfSegments(); j++)
            cmpTrajectoryReference.getPolynomials().set(j, copTrajectoryReference.getPolynomials().get(j));
      }
      cmpTrajectoryReference = transferCMPTrajectories.get(numberOfFootstepsToCopy);
      copTrajectoryReference = transferCoPTrajectories.get(numberOfFootstepsToCopy);
      for (int j = 0; j < copTrajectoryReference.getNumberOfSegments(); j++)
         cmpTrajectoryReference.getPolynomials().set(j, copTrajectoryReference.getPolynomials().get(j));
   }
}
