package us.ihmc.footstepPlanning.scoring;

import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;

import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.BipedalStepScorer;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
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

public class PenalizationHeatmapStepScorer implements BipedalStepScorer
{
   private final BipedalFootstepPlannerParameters footstepPlannerParameters;

   private final YoFrameVector2d forwardPenalizationVector;
   private final YoFrameVector2d backwardPenalizationVector;
   private final YoFrameVector2d inwardPenalizationVector;
   private final YoFrameVector2d outwardPenalizationVector;
   private final YoFrameVector upwardPenalizationVector;
   private final YoFrameVector downwardPenalizationVector;
   private final YoFrameVector2d goalProgressAwardVector;

   private final DoubleYoVariable forwardPenalizationWeight;
   private final DoubleYoVariable backwardPenalizationWeight;
   private final DoubleYoVariable inwardPenalizationWeight;
   private final DoubleYoVariable outwardPenalizationWeight;
   private final DoubleYoVariable upwardPenalizationWeight;
   private final DoubleYoVariable downwardPenalizationWeight;
   private final DoubleYoVariable stancePitchDownwardPenalizationWeight;
   private final DoubleYoVariable angularPenalizationWeight;
   private final DoubleYoVariable goalProgressAwardWeight;
   private final DoubleYoVariable negativeFootholdLinearPenalization;

   private final YoFrameVector idealToCandidateVector;
   private final YoFrameOrientation idealToCandidateOrientation;

   private final FrameVector tempFrameVectorForDot;

   public PenalizationHeatmapStepScorer(YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsRegistry,
                                        BipedalFootstepPlannerParameters footstepPlannerParameters)
   {
      this.footstepPlannerParameters = footstepPlannerParameters;

      String prefix = "footstepScorer";
      forwardPenalizationVector = new YoFrameVector2d(prefix + "ForwardPenalizationVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      backwardPenalizationVector = new YoFrameVector2d(prefix + "BackwardPenalizationVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      inwardPenalizationVector = new YoFrameVector2d(prefix + "InwardPenalizationVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      outwardPenalizationVector = new YoFrameVector2d(prefix + "OutwardPenalizationVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      upwardPenalizationVector = new YoFrameVector(prefix + "UpwardPenalizationVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      downwardPenalizationVector = new YoFrameVector(prefix + "DownwardPenalizationVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      goalProgressAwardVector = new YoFrameVector2d(prefix + "GoalProgressAwardVector", ReferenceFrame.getWorldFrame(), parentRegistry);

      forwardPenalizationWeight = new DoubleYoVariable(prefix + "ForwardPenalizationWeight", parentRegistry);
      backwardPenalizationWeight = new DoubleYoVariable(prefix + "BackwardPenalizationWeight", parentRegistry);
      inwardPenalizationWeight = new DoubleYoVariable(prefix + "InwardPenalizationWeight", parentRegistry);
      outwardPenalizationWeight = new DoubleYoVariable(prefix + "OutwardPenalizationWeight", parentRegistry);
      upwardPenalizationWeight = new DoubleYoVariable(prefix + "UpwardPenalizationWeight", parentRegistry);
      downwardPenalizationWeight = new DoubleYoVariable(prefix + "DownwardPenalizationWeight", parentRegistry);
      stancePitchDownwardPenalizationWeight = new DoubleYoVariable(prefix + "StancePitchDownwardPenalizationWeight", parentRegistry);
      angularPenalizationWeight = new DoubleYoVariable(prefix + "AngularPenalizationWeight", parentRegistry);
      goalProgressAwardWeight = new DoubleYoVariable(prefix + "GoalProgressAwardWeight", parentRegistry);
      negativeFootholdLinearPenalization = new DoubleYoVariable(prefix + "NegativeFootholdLinearPenalization", parentRegistry);

      idealToCandidateVector = new YoFrameVector(prefix + "IdealToCandidateVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      idealToCandidateOrientation = new YoFrameOrientation(prefix + "IdealToCandidateOrientation", ReferenceFrame.getWorldFrame(), parentRegistry);

      tempFrameVectorForDot = new FrameVector();

      setDefaultValues();
   }

   private void setDefaultValues()
   {
      forwardPenalizationWeight.set(-2.5);
      backwardPenalizationWeight.set(-0.8);
      inwardPenalizationWeight.set(-2.0);
      outwardPenalizationWeight.set(-2.0);
      upwardPenalizationWeight.set(-2.0);
      downwardPenalizationWeight.set(-2.0);
      stancePitchDownwardPenalizationWeight.set(-20.0);
      angularPenalizationWeight.set(0.0);
      goalProgressAwardWeight.set(0.0);
      negativeFootholdLinearPenalization.set(-5.0);
   }

   @Override
   public double scoreFootstep(FramePose stanceFoot, FramePose swingStartFoot, FramePose idealFootstep, FramePose candidateFootstep, Point3d goal, double percentageOfFoothold)
   {
      double score = 0.0;

      setXYVectorFromPoseToPoseNormalize(forwardPenalizationVector, swingStartFoot, idealFootstep);
      setXYVectorFromPoseToPoseNormalize(backwardPenalizationVector, idealFootstep, swingStartFoot);
      inwardPenalizationVector.set(forwardPenalizationVector.getY(), -forwardPenalizationVector.getX());
      outwardPenalizationVector.set(-forwardPenalizationVector.getY(), forwardPenalizationVector.getX());
      upwardPenalizationVector.set(0.0, 0.0, 1.0);
      downwardPenalizationVector.set(0.0, 0.0, -1.0);

      setVectorFromPoseToPose(idealToCandidateVector, idealFootstep, candidateFootstep);
      setOrientationFromPoseToPose(idealToCandidateOrientation, idealFootstep, candidateFootstep);

      double downwardPenalizationWeightConsideringStancePitch = downwardPenalizationWeight.getDoubleValue();
      if (stanceFoot.getPitch() < 0)
      {
         downwardPenalizationWeightConsideringStancePitch += -stanceFoot.getPitch() * stancePitchDownwardPenalizationWeight.getDoubleValue();
      }

      score += penalizeCandidateFootstep(forwardPenalizationVector, forwardPenalizationWeight.getDoubleValue());
      score += penalizeCandidateFootstep(backwardPenalizationVector, backwardPenalizationWeight.getDoubleValue());
      score += penalizeCandidateFootstep(inwardPenalizationVector, inwardPenalizationWeight.getDoubleValue());
      score += penalizeCandidateFootstep(outwardPenalizationVector, outwardPenalizationWeight.getDoubleValue());
      score += penalizeCandidateFootstep(upwardPenalizationVector, upwardPenalizationWeight.getDoubleValue());
      score += penalizeCandidateFootstep(downwardPenalizationVector, downwardPenalizationWeightConsideringStancePitch);

      score += angularPenalizationWeight.getDoubleValue() * Math.abs(idealToCandidateOrientation.getYaw().getDoubleValue());
      score += angularPenalizationWeight.getDoubleValue() * Math.abs(idealToCandidateOrientation.getPitch().getDoubleValue());
      score += angularPenalizationWeight.getDoubleValue() * Math.abs(idealToCandidateOrientation.getRoll().getDoubleValue());

      goalProgressAwardVector.set(goal.getX(), goal.getY());
      goalProgressAwardVector.sub(new Vector2d(idealFootstep.getX(), idealFootstep.getY()));

      score += awardCandidateFootstep(goalProgressAwardVector, goalProgressAwardWeight.getDoubleValue());
      
      score += (1.0 - percentageOfFoothold) * negativeFootholdLinearPenalization.getDoubleValue();
      
      return score;
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

   private double awardCandidateFootstep(YoFrameVector2d awardVector, double awardWeight)
   {
      // TODO sqrt??
      double dotProduct = dot3dVectorWith2dVector(idealToCandidateVector, awardVector);
      return awardWeight * dotProduct;
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
      FrameVector2d frameTuple2d = vectorToPack.getFrameTuple2d();
      frameTuple2d.setByProjectionOntoXYPlane(toPose.getFramePointCopy());
      fromPose.checkReferenceFrameMatch(vectorToPack);
      frameTuple2d.sub(fromPose.getX(), fromPose.getY());
      frameTuple2d.normalize();
      vectorToPack.setWithoutChecks(frameTuple2d);
   }
}
