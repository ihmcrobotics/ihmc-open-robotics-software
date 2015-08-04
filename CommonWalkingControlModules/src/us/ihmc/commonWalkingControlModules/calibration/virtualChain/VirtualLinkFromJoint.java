package us.ihmc.commonWalkingControlModules.calibration.virtualChain;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class VirtualLinkFromJoint
{
   protected final Joint joint;
   private double massHere;
   private double massHereOnOut;
   
   protected FrameVector virtualLinkFrameVector;
   protected ReferenceFrame referenceFrame;
   private Vector3d offsetFromParent;
   
   private ArrayList<VirtualLinkFromJoint> childrenObjects = new ArrayList<VirtualLinkFromJoint>();
   
   public VirtualLinkFromJoint(Joint joint)
   {
      this.joint = joint;
   }
   
   public abstract void updateReferenceFrameFromJointAngle();
   
   public void updateReferenceFrameFromJointAngleRecursively()
   {
      this.updateReferenceFrameFromJointAngle();
      
      for (VirtualLinkFromJoint child : childrenObjects)
      {
         child.updateReferenceFrameFromJointAngleRecursively();
      }
   }
   
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }
   
   public void setMassHere(double massHere)
   {
      this.massHere = massHere;
   }
   
   public void setMassHereOnOut(double massHereOnOut)
   {
      this.massHereOnOut = massHereOnOut;
   }
   
   public void setOffsetFromParent(Vector3d offsetFromParent)
   {
      this.offsetFromParent = offsetFromParent;
   }
   
   public void setVirtualLinkFrameVector(FrameVector virtualLinkFrameVector)
   {
      this.virtualLinkFrameVector = virtualLinkFrameVector;
   }
   
   public Joint getJoint()
   {
      return this.joint;
   }
   
   public double getMassHere()
   {
      return massHere;
   }
   
   public double getMassHereOnOut()
   {
      return massHereOnOut;
   }
   
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }
   
   public FrameVector getVirtualLinkFrameVector()
   {
      return virtualLinkFrameVector;
   }
   
   public Vector3d getOffsetFromParent()
   {
      return offsetFromParent;
   }


   public void addChild(VirtualLinkFromJoint childVirtualChainObject)
   {
      childrenObjects.add(childVirtualChainObject);
   }


   public ArrayList<VirtualLinkFromJoint> getChildren()
   {
      return childrenObjects;
   }
   
}
