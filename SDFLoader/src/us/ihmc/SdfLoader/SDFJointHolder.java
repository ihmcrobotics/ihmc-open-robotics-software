package us.ihmc.SdfLoader;

import java.util.ArrayList;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.xmlDescription.SDFJoint;

public class SDFJointHolder
{
   enum JointType
   {
      REVOLUTE
   }

   // Data from SDF
   private final String name;
   private final JointType type;
   private final Vector3d axis;
   private final double upperLimit;
   private final double lowerLimit;
   private final Transform3D transformFromChildLink;

   // Set by loader
   private SDFLinkHolder parent;
   private SDFLinkHolder child;

   //Calculated 
   private Transform3D transformFromParentJoint = null;

   public SDFJointHolder(SDFJoint sdfJoint, SDFLinkHolder parent, SDFLinkHolder child)
   {
      name = sdfJoint.getName();
      String typeString = sdfJoint.getType();

      if (typeString.equalsIgnoreCase("revolute"))
      {
         type = JointType.REVOLUTE;
      }
      else
      {
         throw new RuntimeException("Joint type " + typeString + " not implemented yet");
      }

      axis = SDFConversionsHelper.stringToAxis(sdfJoint.getAxis().getXyz());
      upperLimit = Double.parseDouble(sdfJoint.getAxis().getLimit().getUpper());
      lowerLimit = Double.parseDouble(sdfJoint.getAxis().getLimit().getLower());
      
      transformFromChildLink = SDFConversionsHelper.poseToTransform(sdfJoint.getPose());

      this.parent = parent;
      this.child = child;
      parent.addChild(this);
      child.setJoint(this);
   }
   
   public void calculateTransformToParent()
   {


      Transform3D modelToParentLink = getParent().getTransformFromModelReferenceFrame();
      Transform3D modelToChildLink = getChild().getTransformFromModelReferenceFrame();

      Transform3D parentLinkToParentJoint;
      
      SDFJointHolder parentJoint = parent.getJoint();
      if (parentJoint != null)
      {
         parentLinkToParentJoint = parentJoint.getTransformFromChildLink();
      }
      else
      {
         parentLinkToParentJoint = new Transform3D();
      }

      Transform3D modelToParentJoint = new Transform3D();
      Transform3D modelToChildJoint = new Transform3D();

      modelToParentJoint.mul(modelToParentLink, parentLinkToParentJoint);
      modelToChildJoint.mul(modelToChildLink, transformFromChildLink);

      Transform3D parentJointToModel = new Transform3D();
      parentJointToModel.invert(modelToParentJoint);

      Transform3D parentJointToChildJoint = new Transform3D();
      parentJointToChildJoint.mul(parentJointToModel, modelToChildJoint);

      transformFromParentJoint = parentJointToChildJoint;
   }

   public String getName()
   {
      return name;
   }

   public JointType getType()
   {
      return type;
   }

   public Vector3d getAxis()
   {
      return axis;
   }

   public double getUpperLimit()
   {
      return upperLimit;
   }

   public double getLowerLimit()
   {
      return lowerLimit;
   }

   public Transform3D getTransformFromChildLink()
   {
      return transformFromChildLink;
   }

   public SDFLinkHolder getParent()
   {
      return parent;
   }

   public SDFLinkHolder getChild()
   {
      return child;
   }

   public Transform3D getTransformFromParentJoint()
   {
      return transformFromParentJoint;
   }

   public String toString()
   {
      return name;
   }
}
