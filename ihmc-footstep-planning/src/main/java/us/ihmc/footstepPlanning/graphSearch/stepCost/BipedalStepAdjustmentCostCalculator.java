package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BipedalStepAdjustmentCostCalculator implements BipedalStepCostCalculator
{
   private final YoFrameVector2d forwardCostVector;
   private final YoFrameVector2d backwardCostVector;
   private final YoFrameVector2d inwardCostVector;
   private final YoFrameVector2d outwardCostVector;
   private final YoFrameVector upwardCostVector;
   private final YoFrameVector downwardVector;

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

   private final YoFrameVector idealToCandidateVector;
   private final YoFrameOrientation idealToCandidateOrientation;

   private final FrameVector3D tempFrameVectorForDot;

   public BipedalStepAdjustmentCostCalculator(YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsRegistry)
   {
      String prefix = "StepAdjustment";
      forwardCostVector = new YoFrameVector2d(prefix + "ForwardCostVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      backwardCostVector = new YoFrameVector2d(prefix + "BackwardCostVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      inwardCostVector = new YoFrameVector2d(prefix + "InwardCostVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      outwardCostVector = new YoFrameVector2d(prefix + "OutwardCostVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      upwardCostVector = new YoFrameVector(prefix + "UpwardCostVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      downwardVector = new YoFrameVector(prefix + "DownwardCostVector", ReferenceFrame.getWorldFrame(), parentRegistry);

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

      idealToCandidateVector = new YoFrameVector(prefix + "IdealToCandidateVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      idealToCandidateOrientation = new YoFrameOrientation(prefix + "IdealToCandidateOrientation", ReferenceFrame.getWorldFrame(), parentRegistry);

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
      tempFrameVectorForDot.setIncludingFrame(vector2d.getFrameTuple2d(), 0.0);
      return vector3d.dot(tempFrameVectorForDot);
   }

   private void setOrientationFromPoseToPose(YoFrameOrientation frameOrientationToPack, FramePose fromPose, FramePose toPose)
   {
      FrameQuaternion toOrientation = toPose.getFrameOrientationCopy();
      FrameQuaternion fromOrientation = fromPose.getFrameOrientationCopy();
      frameOrientationToPack.getFrameOrientation().difference(toOrientation, fromOrientation);
   }

   private void setVectorFromPoseToPose(YoFrameVector frameVectorToPack, FramePose fromPose, FramePose toPose)
   {
      frameVectorToPack.set(toPose.getFramePointCopy());
      FrameVector3D frameTuple = new FrameVector3D(frameVectorToPack);
      frameTuple.sub(fromPose.getFramePointCopy());
      frameVectorToPack.set(frameTuple);
   }

   private void setXYVectorFromPoseToPoseNormalize(YoFrameVector2d vectorToPack, FramePose fromPose, FramePose toPose)
   {
      if (fromPose.epsilonEquals(toPose, 1e-7, Double.MAX_VALUE))
      {
         vectorToPack.set(fromPose.getReferenceFrame(), 0.0, 0.0);
      }
      else
      {
         FrameVector2D frameTuple2d = vectorToPack.getFrameTuple2d();
         frameTuple2d.set(toPose.getFramePointCopy());
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
