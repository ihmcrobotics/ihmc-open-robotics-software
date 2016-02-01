package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class RootJointPoseCorrectorHelper
{
   private final DoubleYoVariable positionCorrectionAlpha;
   private final DoubleYoVariable orientationCorrectionAlpha;

//   private final Matrix3d compensationMatrix = new Matrix3d();
//   private final Matrix3d rootJointOrientationMatrix = new Matrix3d();

   private final RigidBodyTransform poseCompensationTransform = new RigidBodyTransform();
   private final RigidBodyTransform rootJointPoseTransform = new RigidBodyTransform();

   private final FrameVector positionErrorVector = new FrameVector();
   private final Vector3d positionCompensationVector = new Vector3d();
   
   private final FrameVector orientationErrorVector = new FrameVector();
   private final AxisAngle4d errorAxisAngle = new AxisAngle4d();
   private final AxisAngle4d orientationCompensationAxisAngle = new AxisAngle4d();
   
   private final PoseReferenceFrame rootJointReferenceFrame = new PoseReferenceFrame("helperPoseReferenceFrame", new FramePose()); 
   
   //temporary variables
   private final Vector3d tempVector = new Vector3d();
   private final FrameOrientation orientationError = new FrameOrientation();
   private final FramePoint positionError = new FramePoint();
   
   
   public RootJointPoseCorrectorHelper(String name, YoVariableRegistry registry)
   {
      orientationCorrectionAlpha = new DoubleYoVariable(name + "OrientationCorrectionAlpha", registry);
      orientationCorrectionAlpha.set(0.15);
      positionCorrectionAlpha = new DoubleYoVariable(name + "PositionCorrectionAlpha", registry);
      positionCorrectionAlpha.set(0.15);
   }

   public void compensatePoseError(FramePose rootJointPoseToPack, FramePose poseError)
   {
      ReferenceFrame errorReferenceFrame = poseError.getReferenceFrame();
      rootJointReferenceFrame.setPoseAndUpdate(rootJointPoseToPack);

      poseError.getPoseIncludingFrame(positionError, orientationError);
      
      estimateOrientationCompensation(errorReferenceFrame);
      
      estimatePositionCompensation();

      rootJointPoseToPack.getRigidBodyTransform(rootJointPoseTransform);
      poseCompensationTransform.set(orientationCompensationAxisAngle, positionCompensationVector);
      
      rootJointPoseTransform.multiply(poseCompensationTransform);
      rootJointPoseToPack.setPose(rootJointPoseTransform);
   }

   private void estimatePositionCompensation()
   {
      positionErrorVector.setIncludingFrame(positionError);
      positionErrorVector.changeFrame(rootJointReferenceFrame);
      positionErrorVector.get(positionCompensationVector);
      positionCompensationVector.scale(positionCorrectionAlpha.getDoubleValue());
   }

   private void estimateOrientationCompensation(ReferenceFrame errorReferenceFrame)
   {
      orientationError.getAxisAngle(errorAxisAngle);
      orientationErrorVector.setIncludingFrame(errorReferenceFrame, errorAxisAngle.getX(), errorAxisAngle.getY(), errorAxisAngle.getZ());
      orientationErrorVector.changeFrame(rootJointReferenceFrame);
      
      double angleErrorAmplitude = errorAxisAngle.getAngle();
      angleErrorAmplitude *= orientationCorrectionAlpha.getDoubleValue();
    
      orientationErrorVector.get(tempVector);
      orientationCompensationAxisAngle.set(tempVector, angleErrorAmplitude);
   }
   
   public void setCorrectionAlpha(double orientationCorrectionAlpha, double positionCorrectionAlpha)
   {
      this.orientationCorrectionAlpha.set(orientationCorrectionAlpha);
      this.positionCorrectionAlpha.set(positionCorrectionAlpha);
   }

}
