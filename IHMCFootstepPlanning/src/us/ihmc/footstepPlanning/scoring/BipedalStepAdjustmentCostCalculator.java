package us.ihmc.footstepPlanning.scoring;

import us.ihmc.footstepPlanning.graphSearch.BipedalStepCostCalculator;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class BipedalStepAdjustmentCostCalculator implements BipedalStepCostCalculator
{
   private final YoFrameVector2d forwardCostVector;
   private final YoFrameVector2d backwardCostVector;
   private final YoFrameVector2d inwardCostVector;
   private final YoFrameVector2d outwardCostVector;
   private final YoFrameVector upwardCostVector;
   private final YoFrameVector downwardVector;

   private final DoubleYoVariable forwardCostScalar;
   private final DoubleYoVariable backwardCostScalar;
   private final DoubleYoVariable inwardCostScalar;
   private final DoubleYoVariable outwardCostScalar;
   private final DoubleYoVariable upwardCostScalar;
   private final DoubleYoVariable downwardCostScalar;
   private final DoubleYoVariable stancePitchDownwardCostScalar;
   private final DoubleYoVariable angularCostScalar;
   private final DoubleYoVariable negativeFootholdLinearCostScalar;

   private final DoubleYoVariable footstepBaseCost;

   private final YoFrameVector idealToCandidateVector;
   private final YoFrameOrientation idealToCandidateOrientation;

   private final FrameVector tempFrameVectorForDot;

   public BipedalStepAdjustmentCostCalculator(YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsRegistry)
   {
      String prefix = "StepAdjustment";
      forwardCostVector = new YoFrameVector2d(prefix + "ForwardCostVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      backwardCostVector = new YoFrameVector2d(prefix + "BackwardCostVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      inwardCostVector = new YoFrameVector2d(prefix + "InwardCostVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      outwardCostVector = new YoFrameVector2d(prefix + "OutwardCostVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      upwardCostVector = new YoFrameVector(prefix + "UpwardCostVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      downwardVector = new YoFrameVector(prefix + "DownwardCostVector", ReferenceFrame.getWorldFrame(), parentRegistry);

      forwardCostScalar = new DoubleYoVariable(prefix + "ForwardCostScalar", parentRegistry);
      backwardCostScalar = new DoubleYoVariable(prefix + "BackwardCostScalar", parentRegistry);
      inwardCostScalar = new DoubleYoVariable(prefix + "InwardCostScalar", parentRegistry);
      outwardCostScalar = new DoubleYoVariable(prefix + "OutwardCostScalar", parentRegistry);
      upwardCostScalar = new DoubleYoVariable(prefix + "UpwardCostScalar", parentRegistry);
      downwardCostScalar = new DoubleYoVariable(prefix + "DownwardCostScalar", parentRegistry);
      stancePitchDownwardCostScalar = new DoubleYoVariable(prefix + "StancePitchDownwardCostScalar", parentRegistry);
      angularCostScalar = new DoubleYoVariable(prefix + "AngularCostScalar", parentRegistry);
      negativeFootholdLinearCostScalar = new DoubleYoVariable(prefix + "NegativeFootholdLinearCostScalar", parentRegistry);

      footstepBaseCost = new DoubleYoVariable(prefix + "FootstepBaseCost", parentRegistry);

      idealToCandidateVector = new YoFrameVector(prefix + "IdealToCandidateVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      idealToCandidateOrientation = new YoFrameOrientation(prefix + "IdealToCandidateOrientation", ReferenceFrame.getWorldFrame(), parentRegistry);

      tempFrameVectorForDot = new FrameVector();

      setDefaultValues();
   }

   private void setDefaultValues()
   {
      forwardCostScalar.set(2.5);
      backwardCostScalar.set(0.8);
      inwardCostScalar.set(2.0);
      outwardCostScalar.set(2.5);
      upwardCostScalar.set(2.0);
      downwardCostScalar.set(0.0); //2.0);
      stancePitchDownwardCostScalar.set(20.0);
      angularCostScalar.set(0.0);
      negativeFootholdLinearCostScalar.set(5.0);

      footstepBaseCost.set(0.3);
   }

   @Override
   public double calculateCost(FramePose stanceFoot, FramePose swingStartFoot, FramePose idealFootstep, FramePose candidateFootstep, double percentageOfFoothold)
   {
      double cost = footstepBaseCost.getDoubleValue();

      setXYVectorFromPoseToPoseNormalize(forwardCostVector, swingStartFoot, idealFootstep);
      setXYVectorFromPoseToPoseNormalize(backwardCostVector, idealFootstep, swingStartFoot);
      inwardCostVector.set(forwardCostVector.getY(), -forwardCostVector.getX());
      outwardCostVector.set(-forwardCostVector.getY(), forwardCostVector.getX());
      upwardCostVector.set(0.0, 0.0, 1.0);
      downwardVector.set(0.0, 0.0, -1.0);

      setVectorFromPoseToPose(idealToCandidateVector, idealFootstep, candidateFootstep);
      setOrientationFromPoseToPose(idealToCandidateOrientation, idealFootstep, candidateFootstep);

      double downwardPenalizationWeightConsideringStancePitch = downwardCostScalar.getDoubleValue();
      if (stanceFoot.getPitch() < 0)
      {
         downwardPenalizationWeightConsideringStancePitch += -stanceFoot.getPitch() * stancePitchDownwardCostScalar.getDoubleValue();
      }

      cost += penalizeCandidateFootstep(forwardCostVector, forwardCostScalar.getDoubleValue());
      cost += penalizeCandidateFootstep(backwardCostVector, backwardCostScalar.getDoubleValue());
      cost += penalizeCandidateFootstep(inwardCostVector, inwardCostScalar.getDoubleValue());
      cost += penalizeCandidateFootstep(outwardCostVector, outwardCostScalar.getDoubleValue());
      cost += penalizeCandidateFootstep(upwardCostVector, upwardCostScalar.getDoubleValue());
      cost += penalizeCandidateFootstep(downwardVector, downwardPenalizationWeightConsideringStancePitch);

      cost += angularCostScalar.getDoubleValue() * Math.abs(idealToCandidateOrientation.getYaw().getDoubleValue());
      cost += angularCostScalar.getDoubleValue() * Math.abs(idealToCandidateOrientation.getPitch().getDoubleValue());
      cost += angularCostScalar.getDoubleValue() * Math.abs(idealToCandidateOrientation.getRoll().getDoubleValue());

      cost += (1.0 - percentageOfFoothold) * negativeFootholdLinearCostScalar.getDoubleValue();

      return cost;
   }

   private double penalizeCandidateFootstep(YoFrameVector penalizationVector, double penalizationWeight)
   {
      // TODO sqrt??
      double dotProduct = idealToCandidateVector.dot(penalizationVector.getFrameTuple());
      dotProduct = Math.max(0.0, dotProduct);
      return penalizationWeight * dotProduct;
   }

   private double penalizeCandidateFootstep(YoFrameVector2d penalizationVector, double penalizationWeight)
   {
      // TODO sqrt??
      double dotProduct = dot3dVectorWith2dVector(idealToCandidateVector, penalizationVector);
      dotProduct = Math.max(0.0, dotProduct);
      return penalizationWeight * dotProduct;
   }

   private double dot3dVectorWith2dVector(YoFrameVector vector3d, YoFrameVector2d vector2d)
   {
      tempFrameVectorForDot.setXYIncludingFrame(vector2d.getFrameTuple2d());
      return vector3d.dot(tempFrameVectorForDot);
   }

   private void setOrientationFromPoseToPose(YoFrameOrientation frameOrientationToPack, FramePose fromPose, FramePose toPose)
   {
      FrameOrientation toOrientation = toPose.getFrameOrientationCopy();
      FrameOrientation fromOrientation = fromPose.getFrameOrientationCopy();
      frameOrientationToPack.getFrameOrientation().setOrientationFromOneToTwo(fromOrientation, toOrientation);
   }

   private void setVectorFromPoseToPose(YoFrameVector frameVectorToPack, FramePose fromPose, FramePose toPose)
   {
      frameVectorToPack.set(toPose.getFramePointCopy());
      FrameVector frameTuple = frameVectorToPack.getFrameTuple();
      frameTuple.sub(fromPose.getFramePointCopy());
      frameVectorToPack.setWithoutChecks(frameTuple);
   }

   private void setXYVectorFromPoseToPoseNormalize(YoFrameVector2d vectorToPack, FramePose fromPose, FramePose toPose)
   {
      if (fromPose.epsilonEquals(toPose, 1e-7, Double.MAX_VALUE))
      {
         vectorToPack.set(fromPose.getReferenceFrame(), 0.0, 0.0);
      }
      else
      {
         FrameVector2d frameTuple2d = vectorToPack.getFrameTuple2d();
         frameTuple2d.setByProjectionOntoXYPlane(toPose.getFramePointCopy());
         fromPose.checkReferenceFrameMatch(vectorToPack);
         frameTuple2d.sub(fromPose.getX(), fromPose.getY());
         frameTuple2d.normalize();
         vectorToPack.setWithoutChecks(frameTuple2d);
      }
   }

   public double getStepBaseCost()
   {
      return footstepBaseCost.getDoubleValue();
   }
}
