package us.ihmc.commonWalkingControlModules.calibration.virtualChain;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RevoluteJointReferenceFrame;
import us.ihmc.robotics.screwTheory.SixDoFJointReferenceFrame;

public class VirtualChainConstructorFromARobot
{
   public VirtualLinkFromJoint createVirtualChainTestObject(Robot robot)
   {
      ArrayList<Joint> rootJoints = robot.getRootJoints();
      if (rootJoints.size() != 1) throw new RuntimeException("Requires exactly one root joint");
      
      Joint rootJoint = rootJoints.get(0);

      VirtualLinkFromJoint rootVirtualChain = recursivePopulateVirtualChain(rootJoint, null); 
      
      double totalMass = rootVirtualChain.getMassHereOnOut();
      recursiveScaleDownByTotalMass(rootVirtualChain, totalMass);
      
      return rootVirtualChain;
   }
   
   public static ArrayList<FrameVector> getVirtualChainFrameVectors(VirtualLinkFromJoint rootTestObject)
   {
      ArrayList<FrameVector> ret = new ArrayList<FrameVector>();
      
      recursivelyGetVirtualChainFrameVectors(ret, rootTestObject);
      
      return ret;
   }
   
   private static void recursivelyGetVirtualChainFrameVectors(ArrayList<FrameVector> frameVectors, VirtualLinkFromJoint testObject)
   {
      frameVectors.add(testObject.getVirtualLinkFrameVector());
      
      for (VirtualLinkFromJoint child : testObject.getChildren())
      {
         recursivelyGetVirtualChainFrameVectors(frameVectors, child);
      }
   }
   
   
   private static void recursiveScaleDownByTotalMass(VirtualLinkFromJoint virtualChainTestObject, double totalMass)
   {
      virtualChainTestObject.getVirtualLinkFrameVector().scale(1.0/totalMass);
      
      ArrayList<VirtualLinkFromJoint> children = virtualChainTestObject.getChildren();
      
      for (VirtualLinkFromJoint child : children)
      {
         recursiveScaleDownByTotalMass(child, totalMass);
      }
   }
   
   private static VirtualLinkFromJoint recursivePopulateVirtualChain(Joint joint, VirtualLinkFromJoint parentTestObject)
   {
      VirtualLinkFromJoint virtualChainTestObject;
      
      if (joint.getClass() == PinJoint.class)
      {
         virtualChainTestObject = new VirtualLinkFromPinJoint((PinJoint) joint);
      }
      else if (joint.getClass() == FloatingJoint.class)
      {
         virtualChainTestObject = new VirtualLinkFromFloatingJoint((FloatingJoint) joint);
      }
      else
      {
         throw new RuntimeException("Only Pin and Floating Joints are implemented");
      }

      recursivePopulateVirtualChain(virtualChainTestObject, joint, parentTestObject);
      
      return virtualChainTestObject;
   }
   
   private static void recursivePopulateVirtualChain(VirtualLinkFromJoint virtualChainTestObject, Joint joint, VirtualLinkFromJoint parentTestObject)
   {
      Link link = joint.getLink();
      double mass = link.getMass();
      
      Vector3d comOffset = new Vector3d();
      link.getComOffset(comOffset);
      
      virtualChainTestObject.setMassHere(mass);

      Vector3d offsetFromParent = new Vector3d();
      joint.getOffset(offsetFromParent);
      virtualChainTestObject.setOffsetFromParent(offsetFromParent);
      
      ReferenceFrame parentFrame = ReferenceFrame.getWorldFrame();
      if (parentTestObject != null) 
      {
         parentFrame = parentTestObject.getReferenceFrame();
      }
//      System.out.println("joint = "  + joint.getName() + ", parentFrame = "  + parentFrame);

      
      ReferenceFrame referenceFrame;
      if (joint.getClass() == PinJoint.class)
      {
         PinJoint pinJoint = (PinJoint) joint;
         Vector3d jointAxis = new Vector3d();
         pinJoint.getJointAxis(jointAxis);
         
         ReferenceFrame translationFrame = ReferenceFrame.constructBodyFrameWithUnchangingTranslationFromParent(joint.getName()+"Translation", parentFrame, offsetFromParent);
        
         FrameVector jointAxisFrameVector = new FrameVector(translationFrame, jointAxis);
         referenceFrame = new RevoluteJointReferenceFrame(joint.getName()+"Revolute", translationFrame, jointAxisFrameVector);
      }
      
      else if (joint.getClass() == FloatingJoint.class)
      {
         referenceFrame = new SixDoFJointReferenceFrame(joint.getName()+"SixDoF", parentFrame);
      }
       
      else
      {
         throw new RuntimeException("Should never get here! Only implemented for PinJoints and FloatingJoints.");
      }
      
      virtualChainTestObject.setReferenceFrame(referenceFrame);
      
      FrameVector virtualLinkFrameVector = new FrameVector(referenceFrame, comOffset);
      virtualLinkFrameVector.scale(mass);
            
      ArrayList<Joint> childrenJoints = joint.getChildrenJoints();
      
      double massHereOnOut = mass;
      for (Joint child : childrenJoints)
      {
         VirtualLinkFromJoint childVirtualChainObject = recursivePopulateVirtualChain(child, virtualChainTestObject);
         
         massHereOnOut =  massHereOnOut + childVirtualChainObject.getMassHereOnOut();
         virtualChainTestObject.addChild(childVirtualChainObject);
         
         FrameVector contributionFromMassOutward = new FrameVector(referenceFrame, childVirtualChainObject.getOffsetFromParent());
         contributionFromMassOutward.scale(childVirtualChainObject.getMassHereOnOut());
         
         virtualLinkFrameVector.add(contributionFromMassOutward);
      }  
      
      virtualChainTestObject.setMassHereOnOut(massHereOnOut);
      virtualChainTestObject.setVirtualLinkFrameVector(virtualLinkFrameVector);      
   }

   
   public static ArrayList<ReferenceFrame> getReferenceFrames(VirtualLinkFromJoint rootLink)
   {
      ArrayList<ReferenceFrame> ret = new ArrayList<ReferenceFrame>();
      
      recursivelyGetReferenceFrames(ret, rootLink);
      
      return ret;
   }
   
   private static void recursivelyGetReferenceFrames(ArrayList<ReferenceFrame> referenceFrames, VirtualLinkFromJoint virtualLink)
   {
      referenceFrames.add(virtualLink.getVirtualLinkFrameVector().getReferenceFrame());
      
      for (VirtualLinkFromJoint child : virtualLink.getChildren())
      {
         recursivelyGetReferenceFrames(referenceFrames, child);
      }
   }
   
}
