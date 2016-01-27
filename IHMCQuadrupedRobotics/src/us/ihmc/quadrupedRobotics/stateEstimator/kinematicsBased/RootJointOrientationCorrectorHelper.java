package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.OrientationFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class RootJointOrientationCorrectorHelper
{
   private final DoubleYoVariable correctionAlpha;

   private final FrameVector orientationVector = new FrameVector();
   
   private final Matrix3d compensationMatrix = new Matrix3d();
   private final Matrix3d rootJointOrientationMatrix = new Matrix3d();
   
   private final AxisAngle4d errorAxisAngle = new AxisAngle4d();
   private final AxisAngle4d compensationAxisAngle = new AxisAngle4d();
   
   private final OrientationFrame rootJointReferenceFrame = new OrientationFrame(new FrameOrientation()); 
   
   //temporary variables
   private final Vector3d tempVector = new Vector3d();
   
   public RootJointOrientationCorrectorHelper(String name, YoVariableRegistry registry)
   {
      correctionAlpha = new DoubleYoVariable(name + "OrientationCorrectionAlpha", registry);
      correctionAlpha.set(0.15);
   }

//   public void compensateOrientationError(ReferenceFrame rootJointReferenceFrame, FrameOrientation rootJointOrientation, FrameOrientation orientationError)
   public void compensateOrientationError(FrameOrientation rootJointOrientationToPack, FrameOrientation orientationError)
   {
      ReferenceFrame errorReferenceFrame = orientationError.getReferenceFrame();
      rootJointReferenceFrame.setOrientationAndUpdate(rootJointOrientationToPack);
      orientationError.getAxisAngle(errorAxisAngle);
      orientationVector.setIncludingFrame(errorReferenceFrame, errorAxisAngle.getX(), errorAxisAngle.getY(), errorAxisAngle.getZ());
      orientationVector.changeFrame(rootJointReferenceFrame);
      
      double angleErrorAmplitude = errorAxisAngle.getAngle();
      angleErrorAmplitude *= correctionAlpha.getDoubleValue();
    
      orientationVector.get(tempVector);
      compensationAxisAngle.set(tempVector, angleErrorAmplitude);
      
      rootJointOrientationToPack.getMatrix3d(rootJointOrientationMatrix);
      compensationMatrix.set(compensationAxisAngle);
//      compensationMatrix.transpose();
      
      rootJointOrientationMatrix.mul(compensationMatrix);
      rootJointOrientationToPack.set(rootJointOrientationMatrix);
   }
   
   public void setCorrectionAlpha(double correctionAlpha)
   {
      this.correctionAlpha.set(correctionAlpha);
   }

}
