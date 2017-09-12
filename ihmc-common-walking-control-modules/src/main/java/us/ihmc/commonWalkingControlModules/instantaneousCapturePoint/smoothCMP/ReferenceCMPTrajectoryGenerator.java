package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.AngularMomentumTrajectory;
import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.TorqueTrajectory;
import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.TrajectoryMathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class ReferenceCMPTrajectoryGenerator
{
   private static final boolean tryBySubtracting = false;
   private static final int maxNumberOfCoefficients = 10;
   private static final int maxNumberOfSegments = 5;

   private final List<CMPTrajectory> transferCMPTrajectories = new ArrayList<>();
   private final List<CMPTrajectory> swingCMPTrajectories = new ArrayList<>();
   private final YoDouble verticalGroundReaction;
   private final TrajectoryMathTools trajectoryMathTools;
   private final YoInteger numberOfFootstepsToConsider;

   private double initialTime;
   private int numberOfRegisteredSteps;
   private CMPTrajectory activeTrajectory;

   private final FramePoint3D desiredCMP = new FramePoint3D();
   private final FrameVector3D desiredCMPVelocity = new FrameVector3D();
   private final TorqueTrajectory torqueTrajectory;

   public ReferenceCMPTrajectoryGenerator(String namePrefix, int maxNumberOfFootstepsToConsider, YoInteger numberOfFootstepsToConsider, YoVariableRegistry registry)
   {
      this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         CMPTrajectory transferCMPTrajectory = new CMPTrajectory(maxNumberOfSegments, maxNumberOfCoefficients);
         CMPTrajectory swingCMPTrajectory = new CMPTrajectory(maxNumberOfSegments, maxNumberOfCoefficients);
         transferCMPTrajectories.add(transferCMPTrajectory);
         swingCMPTrajectories.add(swingCMPTrajectory);
      }
      CMPTrajectory transferCMPTrajectory = new CMPTrajectory(maxNumberOfSegments, maxNumberOfCoefficients);
      transferCMPTrajectories.add(transferCMPTrajectory);
      this.torqueTrajectory = new TorqueTrajectory(maxNumberOfSegments, maxNumberOfCoefficients);
      this.verticalGroundReaction = new YoDouble(namePrefix + "CMPTorqueOffsetScalingFactor", registry);
      this.trajectoryMathTools = new TrajectoryMathTools(maxNumberOfCoefficients);
   }
   
   public void setGroundReaction(double z)
   {
      this.verticalGroundReaction.set(z);
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

   public void getPosition(FramePoint3D desiredCMPToPack)
   {
      desiredCMPToPack.setIncludingFrame(desiredCMP);
   }

   public void getPosition(YoFramePoint desiredCMPToPack)
   {
      desiredCMPToPack.set(desiredCMP);
   }

   public void getVelocity(FrameVector3D desiredCMPVelocityToPack)
   {
      desiredCMPVelocityToPack.setIncludingFrame(desiredCMPVelocity);
   }

   public void getVelocity(YoFrameVector desiredCMPVelocityToPack)
   {
      desiredCMPVelocityToPack.set(desiredCMPVelocity);
   }

   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack)
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

   public void initializeForTransfer(double currentTime, List<? extends CoPTrajectory> transferCoPTrajectories,
                                     List<? extends CoPTrajectory> swingCoPTrajectories,
                                     List<? extends AngularMomentumTrajectory> transferAngularMomentumTrajectories,
                                     List<? extends AngularMomentumTrajectory> swingAngularMomentumTrajectories)
   {
      this.initialTime = currentTime;
      setCMPTrajectories(transferCoPTrajectories, swingCoPTrajectories, transferAngularMomentumTrajectories, swingAngularMomentumTrajectories, WalkingTrajectoryType.TRANSFER);
      activeTrajectory = transferCMPTrajectories.get(0);      
   }

   public void initializeForSwing(double currentTime, List<? extends CoPTrajectory> transferCoPTrajectories,
                                  List<? extends CoPTrajectory> swingCoPTrajectories,
                                  List<? extends AngularMomentumTrajectory> transferAngularMomentumTrajectories,
                                  List<? extends AngularMomentumTrajectory> swingAngularMomentumTrajectories)
   {
      this.initialTime = currentTime;
      setCMPTrajectories(transferCoPTrajectories, swingCoPTrajectories, transferAngularMomentumTrajectories, swingAngularMomentumTrajectories, WalkingTrajectoryType.SWING);
      activeTrajectory = swingCMPTrajectories.get(0);
   }

   private void setCMPTrajectories(List<? extends CoPTrajectory> transferCoPTrajectories, List<? extends CoPTrajectory> swingCoPTrajectories,
                                   List<? extends AngularMomentumTrajectory> transferAngularMomentumTrajectories,
                                   List<? extends AngularMomentumTrajectory> swingAngularMomentumTrajectories,  WalkingTrajectoryType phase)
   {
      if (transferAngularMomentumTrajectories == null || swingAngularMomentumTrajectories == null)
      {
         copyCoPTrajectoriesToCMPTrajectories(transferCoPTrajectories, swingCoPTrajectories);
         return;
      }
      int numberOfFootstepsToSet = Math.min(numberOfFootstepsToConsider.getIntegerValue(), numberOfRegisteredSteps);
      int index = 0;
      if(phase == WalkingTrajectoryType.SWING)
      {
         CMPTrajectory cmpTrajectoryReference = swingCMPTrajectories.get(index);
         CoPTrajectory copTrajectoryReference = swingCoPTrajectories.get(index);
         torqueTrajectory.set(swingAngularMomentumTrajectories.get(index));
         torqueTrajectory.scale(1.0/verticalGroundReaction.getDoubleValue());
         if(copTrajectoryReference.getNumberOfSegments() == 0 || torqueTrajectory.getNumberOfSegments() == 0)
         {
            return;
         }
         if (tryBySubtracting)
            torqueTrajectory.scale(-1.0);
         trajectoryMathTools.addSegmentedTrajectories(cmpTrajectoryReference, copTrajectoryReference, torqueTrajectory, Epsilons.ONE_HUNDRED_THOUSANDTH);
         index++;
      }
      
      for ( ;index < numberOfFootstepsToSet; index++)
      {
         CMPTrajectory cmpTrajectoryReference = transferCMPTrajectories.get(index);
         CoPTrajectory copTrajectoryReference = transferCoPTrajectories.get(index);
         torqueTrajectory.set(transferAngularMomentumTrajectories.get(index));
         torqueTrajectory.scale(1.0/verticalGroundReaction.getDoubleValue());
         if(copTrajectoryReference.getNumberOfSegments() == 0 || torqueTrajectory.getNumberOfSegments() == 0)
         {
            return;
         }
         if (tryBySubtracting)
            torqueTrajectory.scale(-1.0);
         trajectoryMathTools.addSegmentedTrajectories(cmpTrajectoryReference, copTrajectoryReference, torqueTrajectory, Epsilons.ONE_HUNDRED_THOUSANDTH);
         cmpTrajectoryReference = swingCMPTrajectories.get(index);
         copTrajectoryReference = swingCoPTrajectories.get(index);
         torqueTrajectory.set(swingAngularMomentumTrajectories.get(index));
         torqueTrajectory.scale(1.0/verticalGroundReaction.getDoubleValue());
         if(copTrajectoryReference.getNumberOfSegments() == 0 || torqueTrajectory.getNumberOfSegments() == 0)
         {
            return;
         }
         if (tryBySubtracting)
            torqueTrajectory.scale(-1.0);
         trajectoryMathTools.addSegmentedTrajectories(cmpTrajectoryReference, copTrajectoryReference, torqueTrajectory, Epsilons.ONE_HUNDRED_THOUSANDTH);
      }
      CMPTrajectory cmpTrajectoryReference = transferCMPTrajectories.get(numberOfFootstepsToSet);
      CoPTrajectory copTrajectoryReference = transferCoPTrajectories.get(numberOfFootstepsToSet);
      torqueTrajectory.set(transferAngularMomentumTrajectories.get(numberOfFootstepsToSet));
      torqueTrajectory.scale(1.0/verticalGroundReaction.getDoubleValue());
      if(copTrajectoryReference.getNumberOfSegments() == 0 || torqueTrajectory.getNumberOfSegments() == 0)
         return;
      if (tryBySubtracting)
         torqueTrajectory.scale(-1.0);
      trajectoryMathTools.addSegmentedTrajectories(cmpTrajectoryReference, copTrajectoryReference, torqueTrajectory, Epsilons.ONE_HUNDRED_THOUSANDTH);
   }

   private void copyCoPTrajectoriesToCMPTrajectories(List<? extends CoPTrajectory> transferCoPTrajectories,
                                                     List<? extends CoPTrajectory> swingCoPTrajectories)
   {
      int numberOfFootstepsToCopy = Math.min(numberOfFootstepsToConsider.getIntegerValue(), numberOfRegisteredSteps);
      for (int i = 0; i < numberOfFootstepsToCopy; i++)
      {
         CMPTrajectory cmpTrajectoryReference = transferCMPTrajectories.get(i);
         CoPTrajectory copTrajectoryReference = transferCoPTrajectories.get(i);
         for (int j = 0; j < copTrajectoryReference.getNumberOfSegments(); j++)
            cmpTrajectoryReference.getSegment(j).set(copTrajectoryReference.getSegments().get(j));
         cmpTrajectoryReference.setNumberOfSegments(copTrajectoryReference.getNumberOfSegments());
         cmpTrajectoryReference = swingCMPTrajectories.get(i);
         copTrajectoryReference = swingCoPTrajectories.get(i);
         for (int j = 0; j < copTrajectoryReference.getNumberOfSegments(); j++)
            cmpTrajectoryReference.getSegment(j).set(copTrajectoryReference.getSegments().get(j));
         cmpTrajectoryReference.setNumberOfSegments(copTrajectoryReference.getNumberOfSegments());
      }
      CMPTrajectory cmpTrajectoryReference = transferCMPTrajectories.get(numberOfFootstepsToCopy);
      CoPTrajectory copTrajectoryReference = transferCoPTrajectories.get(numberOfFootstepsToCopy);
      for (int j = 0; j < copTrajectoryReference.getNumberOfSegments(); j++)
         cmpTrajectoryReference.getSegment(j).set(copTrajectoryReference.getSegments().get(j));
      cmpTrajectoryReference.setNumberOfSegments(copTrajectoryReference.getNumberOfSegments());
   }
}
