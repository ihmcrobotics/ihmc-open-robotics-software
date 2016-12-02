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
   private final DoubleYoVariable angularPenalizationWeight;
   private final DoubleYoVariable goalProgressAwardWeight;

   private final YoFrameVector idealToCandidateVector;
   private final YoFrameOrientation idealToCandidateOrientation;

   private final FrameVector tempFrameVectorForDot;

   public PenalizationHeatmapStepScorer(YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsRegistry, BipedalFootstepPlannerParameters footstepPlannerParameters)
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
      angularPenalizationWeight = new DoubleYoVariable(prefix + "AngularPenalizationWeight", parentRegistry);
      goalProgressAwardWeight = new DoubleYoVariable(prefix + "GoalProgressAwardWeight", parentRegistry);

      idealToCandidateVector = new YoFrameVector(prefix + "IdealToCandidateVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      idealToCandidateOrientation = new YoFrameOrientation(prefix + "IdealToCandidateOrientation", ReferenceFrame.getWorldFrame(), parentRegistry);

      tempFrameVectorForDot = new FrameVector();

      setDefaultValues();
   }

   private void setDefaultValues()
   {
      forwardPenalizationWeight.set(-2.5);
      backwardPenalizationWeight.set(-0.4);
      inwardPenalizationWeight.set(-1.4);
      outwardPenalizationWeight.set(-1.4);
      upwardPenalizationWeight.set(0.0);
      downwardPenalizationWeight.set(2.0);
      angularPenalizationWeight.set(0.0);
      goalProgressAwardWeight.set(0.1);
   }

   @Override
   public double scoreFootstep(FramePose stanceFoot, FramePose swingStartFoot, FramePose idealFootstep, FramePose candidateFootstep, Point3d goal)
   {
      double score = 0.0;

      setXYVectorFromPoseToPoseNormalize(forwardPenalizationVector, swingStartFoot, idealFootstep);
      setXYVectorFromPoseToPoseNormalize(backwardPenalizationVector, idealFootstep, swingStartFoot);
      inwardPenalizationVector.set(forwardPenalizationVector.getY(), -forwardPenalizationVector.getX());
      outwardPenalizationVector.set(-forwardPenalizationVector.getY(), forwardPenalizationVector.getX());
      upwardPenalizationVector.set(0.0, 0.0, -1.0);
      downwardPenalizationVector.set(0.0, 0.0, 1.0);

      setVectorFromPoseToPose(idealToCandidateVector, idealFootstep, candidateFootstep);
      setOrientationFromPoseToPose(idealToCandidateOrientation, idealFootstep, candidateFootstep);

      score += penalizeCandidateFootstep(forwardPenalizationVector, forwardPenalizationWeight);
      score += penalizeCandidateFootstep(backwardPenalizationVector, backwardPenalizationWeight);
      score += penalizeCandidateFootstep(inwardPenalizationVector, inwardPenalizationWeight);
      score += penalizeCandidateFootstep(outwardPenalizationVector, outwardPenalizationWeight);
      score += penalizeCandidateFootstep(upwardPenalizationVector, upwardPenalizationWeight);
      score += penalizeCandidateFootstep(downwardPenalizationVector, downwardPenalizationWeight);

      score += angularPenalizationWeight.getDoubleValue() * idealToCandidateOrientation.getYaw().getDoubleValue();
      score += angularPenalizationWeight.getDoubleValue() * idealToCandidateOrientation.getPitch().getDoubleValue();
      score += angularPenalizationWeight.getDoubleValue() * idealToCandidateOrientation.getRoll().getDoubleValue();

      goalProgressAwardVector.set(goal.getX(), goal.getY());
      goalProgressAwardVector.sub(new Vector2d(idealFootstep.getX(), idealFootstep.getY()));

      score += awardCandidateFootstep(goalProgressAwardVector, goalProgressAwardWeight);

      return score;
   }

   private double penalizeCandidateFootstep(YoFrameVector penalizationVector, DoubleYoVariable penalizationWeight)
   {
      // TODO sqrt??
      double dotProduct = idealToCandidateVector.dot(penalizationVector.getFrameTuple());
      dotProduct = Math.max(0.0, dotProduct);
      return penalizationWeight.getDoubleValue() * dotProduct;
   }

   private double penalizeCandidateFootstep(YoFrameVector2d penalizationVector, DoubleYoVariable penalizationWeight)
   {
      // TODO sqrt??
      double dotProduct = dot3dVectorWith2dVector(idealToCandidateVector, penalizationVector);
      dotProduct = Math.max(0.0, dotProduct);
      return penalizationWeight.getDoubleValue() * dotProduct;
   }

   private double awardCandidateFootstep(YoFrameVector2d awardVector, DoubleYoVariable awardWeight)
   {
      // TODO sqrt??
      double dotProduct = dot3dVectorWith2dVector(idealToCandidateVector, awardVector);
      return awardWeight.getDoubleValue() * dotProduct;
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
