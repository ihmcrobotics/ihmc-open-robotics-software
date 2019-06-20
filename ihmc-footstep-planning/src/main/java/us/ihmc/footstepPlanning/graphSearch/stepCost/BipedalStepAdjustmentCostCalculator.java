package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tools.YawPitchRollTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

public class BipedalStepAdjustmentCostCalculator implements BipedalStepCostCalculator
{
   private final YoFrameVector2D forwardCostVector;
   private final YoFrameVector2D backwardCostVector;
   private final YoFrameVector2D inwardCostVector;
   private final YoFrameVector2D outwardCostVector;
   private final YoFrameVector3D upwardCostVector;
   private final YoFrameVector3D downwardVector;

   private final YoDouble forwardCostScalar;
   private final YoDouble backwardCostScalar;
   private final YoDouble inwardCostScalar;
   private final YoDouble outwardCostScalar;
   private final YoDouble upwardCostScalar;
   private final YoDouble downwardCostScalar;
   private final YoDouble stancePitchDownwardCostScalar;
   private final YoDouble angularCostScalar;
   private final YoDouble negativeFootholdLinearCostScalar;

   private final YoDouble footstepBaseCost;

   private final YoFrameVector3D idealToCandidateVector;
   private final YoFrameYawPitchRoll idealToCandidateOrientation;

   private final FrameVector3D tempFrameVectorForDot;

   public BipedalStepAdjustmentCostCalculator(YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsRegistry)
   {
      String prefix = "StepAdjustment";
      forwardCostVector = new YoFrameVector2D(prefix + "ForwardCostVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      backwardCostVector = new YoFrameVector2D(prefix + "BackwardCostVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      inwardCostVector = new YoFrameVector2D(prefix + "InwardCostVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      outwardCostVector = new YoFrameVector2D(prefix + "OutwardCostVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      upwardCostVector = new YoFrameVector3D(prefix + "UpwardCostVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      downwardVector = new YoFrameVector3D(prefix + "DownwardCostVector", ReferenceFrame.getWorldFrame(), parentRegistry);

      forwardCostScalar = new YoDouble(prefix + "ForwardCostScalar", parentRegistry);
      backwardCostScalar = new YoDouble(prefix + "BackwardCostScalar", parentRegistry);
      inwardCostScalar = new YoDouble(prefix + "InwardCostScalar", parentRegistry);
      outwardCostScalar = new YoDouble(prefix + "OutwardCostScalar", parentRegistry);
      upwardCostScalar = new YoDouble(prefix + "UpwardCostScalar", parentRegistry);
      downwardCostScalar = new YoDouble(prefix + "DownwardCostScalar", parentRegistry);
      stancePitchDownwardCostScalar = new YoDouble(prefix + "StancePitchDownwardCostScalar", parentRegistry);
      angularCostScalar = new YoDouble(prefix + "AngularCostScalar", parentRegistry);
      negativeFootholdLinearCostScalar = new YoDouble(prefix + "NegativeFootholdLinearCostScalar", parentRegistry);

      footstepBaseCost = new YoDouble(prefix + "FootstepBaseCost", parentRegistry);

      idealToCandidateVector = new YoFrameVector3D(prefix + "IdealToCandidateVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      idealToCandidateOrientation = new YoFrameYawPitchRoll(prefix + "IdealToCandidateOrientation", ReferenceFrame.getWorldFrame(), parentRegistry);

      tempFrameVectorForDot = new FrameVector3D();

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
   public double calculateCost(FramePose3D stanceFoot, FramePose3D swingStartFoot, FramePose3D idealFootstep, FramePose3D candidateFootstep, double percentageOfFoothold)
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

      cost += angularCostScalar.getDoubleValue() * Math.abs(idealToCandidateOrientation.getYaw());
      cost += angularCostScalar.getDoubleValue() * Math.abs(idealToCandidateOrientation.getPitch());
      cost += angularCostScalar.getDoubleValue() * Math.abs(idealToCandidateOrientation.getRoll());

      cost += (1.0 - percentageOfFoothold) * negativeFootholdLinearCostScalar.getDoubleValue();

      return cost;
   }

   private double penalizeCandidateFootstep(YoFrameVector3D penalizationVector, double penalizationWeight)
   {
      // TODO sqrt??
      double dotProduct = idealToCandidateVector.dot(penalizationVector);
      dotProduct = Math.max(0.0, dotProduct);
      return penalizationWeight * dotProduct;
   }

   private double penalizeCandidateFootstep(YoFrameVector2D penalizationVector, double penalizationWeight)
   {
      // TODO sqrt??
      double dotProduct = dot3dVectorWith2dVector(idealToCandidateVector, penalizationVector);
      dotProduct = Math.max(0.0, dotProduct);
      return penalizationWeight * dotProduct;
   }

   private double dot3dVectorWith2dVector(YoFrameVector3D vector3d, YoFrameVector2D vector2d)
   {
      tempFrameVectorForDot.setIncludingFrame(vector2d, 0.0);
      return vector3d.dot(tempFrameVectorForDot);
   }

   private void setOrientationFromPoseToPose(YoFrameYawPitchRoll frameOrientationToPack, FramePose3D fromPose, FramePose3D toPose)
   {
      YawPitchRollTools.multiply(toPose.getOrientation(), true, fromPose.getOrientation(), false, frameOrientationToPack);
   }

   private void setVectorFromPoseToPose(YoFrameVector3D frameVectorToPack, FramePose3D fromPose, FramePose3D toPose)
   {
      frameVectorToPack.set(toPose.getPosition());
      FrameVector3D frameTuple = new FrameVector3D(frameVectorToPack);
      frameTuple.sub(fromPose.getPosition());
      frameVectorToPack.set(frameTuple);
   }

   private void setXYVectorFromPoseToPoseNormalize(YoFrameVector2D vectorToPack, FramePose3D fromPose, FramePose3D toPose)
   {
      if (fromPose.getPosition().epsilonEquals(toPose.getPosition(), 1e-7))
      {
         vectorToPack.set(fromPose.getReferenceFrame(), 0.0, 0.0);
      }
      else
      {
         FrameVector2D frameTuple2d = new FrameVector2D(vectorToPack);
         frameTuple2d.set(toPose.getPosition());
         fromPose.checkReferenceFrameMatch(vectorToPack);
         frameTuple2d.sub(fromPose.getX(), fromPose.getY());
         frameTuple2d.normalize();
         vectorToPack.set((Tuple2DReadOnly) frameTuple2d);
      }
   }

   public double getStepBaseCost()
   {
      return footstepBaseCost.getDoubleValue();
   }
}
