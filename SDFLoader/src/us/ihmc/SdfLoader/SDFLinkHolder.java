package us.ihmc.SdfLoader;

import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.xmlDescription.SDFLink;
import us.ihmc.SdfLoader.xmlDescription.SDFVisual;
import us.ihmc.utilities.math.MatrixTools;

public class SDFLinkHolder
{
   
   // From SDF File
   private final String name;
   private final Transform3D transformToModelReferenceFrame;
   private final double mass;
   private final Transform3D inertialFrameWithRespectToLinkFrame;
   private final Matrix3d inertia;
   
   private double contactKp = 0.0;
   private double contactKd = 0.0;
   private double contactMaxVel = 0.0;;
   
   private final List<SDFVisual> visuals;
   
   // Set by loader
   private SDFJointHolder joint = null;
   private final ArrayList<SDFJointHolder> childeren = new ArrayList<SDFJointHolder>();

   
   // Calculated
   private final Vector3d CoMOffset = new Vector3d();

   public SDFLinkHolder(SDFLink sdfLink)
   {
     name = sdfLink.getName();
     transformToModelReferenceFrame = SDFConversionsHelper.poseToTransform(sdfLink.getPose());
     mass = Double.parseDouble(sdfLink.getInertial().getMass());
     inertialFrameWithRespectToLinkFrame = SDFConversionsHelper.poseToTransform(sdfLink.getInertial().getPose());
     inertia = SDFConversionsHelper.sdfInertiaToMatrix3d(sdfLink.getInertial().getInertia());
     visuals = sdfLink.getVisuals();
     if(sdfLink.getCollision() != null)
     {
        if(sdfLink.getCollision().getSurface() != null)
        {
           if(sdfLink.getCollision().getSurface().getContact() != null)
           {
              if(sdfLink.getCollision().getSurface().getContact().getOde() != null)
              {
                 if(sdfLink.getCollision().getSurface().getContact().getOde().getKp() != null)
                    contactKp = Double.parseDouble(sdfLink.getCollision().getSurface().getContact().getOde().getKp());
                 if(sdfLink.getCollision().getSurface().getContact().getOde().getKd() != null)
                    contactKd = Double.parseDouble(sdfLink.getCollision().getSurface().getContact().getOde().getKd());
                 if(sdfLink.getCollision().getSurface().getContact().getOde().getMaxVel() != null)
                    contactMaxVel = Double.parseDouble(sdfLink.getCollision().getSurface().getContact().getOde().getMaxVel());
                 
              }
           }
        }
     }
   }

   public Transform3D getTransformFromModelReferenceFrame()
   {
      return transformToModelReferenceFrame;
   }
   
   public void calculateCoMOffset()
   {
      
      Transform3D modelFrameToJointFrame = new Transform3D();
      if(joint != null)
      {
         modelFrameToJointFrame.set(joint.getTransformFromChildLink()); // H_4^3
      }
      Transform3D modelFrameToInertialFrame = inertialFrameWithRespectToLinkFrame; // H_4^5
      Transform3D jointFrameToModelFrame = new Transform3D(); // H_3^4
      jointFrameToModelFrame.invert(modelFrameToJointFrame);
      
      Transform3D jointFrameToInertialFrame = new Transform3D();
      jointFrameToInertialFrame.mul(jointFrameToModelFrame, modelFrameToInertialFrame);
      
      Vector3d CoMOffset = new Vector3d();
      Matrix3d inertialFrameRotation = new Matrix3d();
      
      jointFrameToInertialFrame.get(inertialFrameRotation, CoMOffset);
      
      if(!inertialFrameRotation.epsilonEquals(MatrixTools.IDENTITY, 1e-5))
      {
         System.err.println("Warning: Non-zero rotation of the inertial matrix on link " + name);
      }
      
      this.CoMOffset.set(CoMOffset);
   }

   public ArrayList<SDFJointHolder> getChilderen()
   {
      return childeren;
   }
   
   public void addChild(SDFJointHolder child)
   {
      childeren.add(child);
   }
   
   public SDFJointHolder getJoint()
   {
      return joint;
   }
   
   public void setJoint(SDFJointHolder joint)
   {
      this.joint = joint;
   }
   
   public String toString()
   {
      return name;
   }

   public String getName()
   {
      return name;
   }

   public List<SDFVisual> getVisuals()
   {
     return visuals;
   }

   public double getMass()
   {
      return mass;
   }

   public Matrix3d getInertia()
   {
      return inertia;
   }

   public Vector3d getCoMOffset()
   {
      return CoMOffset;
   }

   public double getContactKp()
   {
      return contactKp;
   }

   public double getContactKd()
   {
      return contactKd;
   }

   public double getContactMaxVel()
   {
      return contactMaxVel;
   }
   
   
}
