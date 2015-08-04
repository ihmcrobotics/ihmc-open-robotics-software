package us.ihmc.commonWalkingControlModules.calibration.virtualChain;

import java.util.ArrayList;

import javax.vecmath.Matrix3d;

import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class VirtualChainDataPoint
{
   private final FramePoint2d centerOfMassProjection;
//   private final FramePoint centerOfMassProjection;
   private final ArrayList<Matrix3d> rotationMatrices;

   public VirtualChainDataPoint(ReferenceFrame baseFrame, ArrayList<ReferenceFrame> referenceFrames, FramePoint2d centerOfMassProjection)
//   public VirtualChainDataPoint(ReferenceFrame baseFrame, ArrayList<ReferenceFrame> referenceFrames, FramePoint centerOfMassProjection)
   {
      centerOfMassProjection.checkReferenceFrameMatch(baseFrame);
      
      if (!centerOfMassProjection.getReferenceFrame().isZupFrame())
      {
         throw new RuntimeException("centerOfMassProjection is Not a ZUp reference frame!");
      }

      this.centerOfMassProjection = new FramePoint2d(centerOfMassProjection);
//      centerOfMassProjection.changeFrame(baseFrame);

      rotationMatrices = new ArrayList<Matrix3d>();

      for (int i = 0; i < referenceFrames.size(); i++)
      {
         ReferenceFrame virtualChainFrame = referenceFrames.get(i);

         RigidBodyTransform transform3D = virtualChainFrame.getTransformToDesiredFrame(baseFrame);

         Matrix3d rotationMatrix = new Matrix3d();
         transform3D.get(rotationMatrix);

         rotationMatrices.add(rotationMatrix);
      }
   }
   
   public int getNumberOfDegreesOfFreedom()
   {
      return rotationMatrices.size();
   }

   public FramePoint2d getCenterOfMassProjection()
   {
      return centerOfMassProjection;
   }

   public ArrayList<Matrix3d> getRotationMatrices()
   {
      return rotationMatrices;
   }
   
   public String toString()
   {
      String ret = "";
      ret = ret + "CoM Projection: " + centerOfMassProjection;
      
      return ret;
   }
  

}

