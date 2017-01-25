package us.ihmc.modelFileLoaders.SdfLoader;

import java.io.IOException;
import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.modelFileLoaders.ModelFileLoaderConversionsHelper;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFJoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.io.printing.PrintTools;

public class SDFJointHolder
{
   public static final boolean DEBUG = false;
   
   // Data from SDF
   private final String name;
   private final SDFJointType type;
   private final Vector3d axisInModelFrame;
   
   private final boolean hasLimits;
   private final double upperLimit;
   private final double lowerLimit;
   
   private final double effortLimit;
   private final double velocityLimit;
   
   
   private final RigidBodyTransform transformFromChildLink;
   private double damping = 0.0;
   private double friction = 0.0;
   
   // Extra data
   private final ArrayList<SDFForceSensor> forceSensors = new ArrayList<>();
   private final ArrayList<SDFContactSensor> contactSensors = new ArrayList<>();
   
   // Set by loader
   private SDFLinkHolder parentLinkHolder;
   private SDFLinkHolder childLinkHolder;

   //Calculated 
   private RigidBodyTransform transformToParentJoint = null;
   private final Matrix3d linkRotation = new Matrix3d();
   private final Vector3d offsetFromParentJoint = new Vector3d();
   private final Vector3d axisInParentFrame = new Vector3d();
   private final Vector3d axisInJointFrame = new Vector3d();
   
   private double contactKp;
   private double contactKd;
   private double maxVel;

   public SDFJointHolder(SDFJoint sdfJoint, SDFLinkHolder parent, SDFLinkHolder child)   throws IOException
   {
      name = createValidVariableName(sdfJoint.getName());
      String typeString = sdfJoint.getType();

      if (typeString.equalsIgnoreCase("revolute"))
      {
         type = SDFJointType.REVOLUTE;
      }
      else if (typeString.equalsIgnoreCase("prismatic"))
      {
         type = SDFJointType.PRISMATIC;
      }
      else
      {
         throw new IOException("Joint type " + typeString + " not implemented yet");
      }

      axisInModelFrame = ModelFileLoaderConversionsHelper.stringToNormalizedVector3d(sdfJoint.getAxis().getXyz());
      
      if(sdfJoint.getAxis().getLimit() != null)
      {
         double sdfUpperLimit = Double.parseDouble(sdfJoint.getAxis().getLimit().getUpper());
         double sdfLowerLimit = Double.parseDouble(sdfJoint.getAxis().getLimit().getLower());
         
         if (sdfUpperLimit > sdfLowerLimit)
         {
            hasLimits = true;
            upperLimit = sdfUpperLimit;
            lowerLimit = sdfLowerLimit;
         }
         else
         {
            hasLimits = true;
            upperLimit = Double.POSITIVE_INFINITY;
            lowerLimit = Double.NEGATIVE_INFINITY;
            PrintTools.debug(DEBUG, this, sdfJoint.getName() + " has invalid joint limits.  LowerLimit = " + sdfLowerLimit + ", UpperLimit = " + sdfUpperLimit + ".  Using LowerLimit = " + lowerLimit + ", UpperLimit = " + upperLimit + " instead.");
         }

         if(sdfJoint.getAxis().getLimit().getVelocity() != null)
         {
            velocityLimit = Double.parseDouble(sdfJoint.getAxis().getLimit().getVelocity());
         }
         else
         {
            velocityLimit = Double.NaN;
         }
         
         if(sdfJoint.getAxis().getLimit().getEffort() != null)
         {
            effortLimit = Double.parseDouble(sdfJoint.getAxis().getLimit().getEffort());
         }
         else
         {
            effortLimit = Double.NaN;
         }
      }
      else
      {
         hasLimits = false;
         upperLimit = Double.POSITIVE_INFINITY;
         lowerLimit = Double.NEGATIVE_INFINITY;
         
         velocityLimit = Double.NaN;
         effortLimit = Double.NaN;
      }
      
      if(sdfJoint.getAxis().getDynamics() != null)
      {
         if(sdfJoint.getAxis().getDynamics().getFriction() != null)
         {
            friction = Double.parseDouble(sdfJoint.getAxis().getDynamics().getFriction());
         }
         if(sdfJoint.getAxis().getDynamics().getDamping() != null)
         {
            damping = Double.parseDouble(sdfJoint.getAxis().getDynamics().getDamping());
         }
      }
      
      transformFromChildLink = ModelFileLoaderConversionsHelper.poseToTransform(sdfJoint.getPose());

      if(parent == null || child == null)
      {
         throw new IOException("Cannot make joint with null parent or child links, joint name is " + sdfJoint.getName());
      }
      
      this.parentLinkHolder = parent;
      this.childLinkHolder = child;
      parent.addChild(this);
      child.setJoint(this);
      
      calculateContactGains();
   }
   
   public static String createValidVariableName(String name)
   {
      name = name.trim().replaceAll("[//[//]///]", "");
      return name;
   }
   
   private void calculateContactGains()
   {
      double parentKp = parentLinkHolder.getContactKp();
      double childKp = childLinkHolder.getContactKp();
      
      if(Math.abs(parentKp) > 1e-3 && Math.abs(childKp) > 1e-3)
      {
         contactKp = 1.0 / (( 1.0 / parentKp ) + (1.0 / childKp));
      }
      else if (Math.abs(parentKp) > 1e-3)
      {
         contactKp = parentKp;
      }
      else if (Math.abs(childKp) > 1e-3)
      {
         contactKp = childKp;
      }
      
      contactKd = parentLinkHolder.getContactKd() + childLinkHolder.getContactKd();
      
      maxVel = Math.min(parentLinkHolder.getContactMaxVel(), childLinkHolder.getContactMaxVel());
      
   }

   public void calculateTransformToParentJoint()
   {


      RigidBodyTransform modelToParentLink = getParentLinkHolder().getTransformFromModelReferenceFrame();
      RigidBodyTransform modelToChildLink = getChildLinkHolder().getTransformFromModelReferenceFrame();

      RigidBodyTransform rotationTransform = new RigidBodyTransform();
      RigidBodyTransform parentLinkToParentJoint;
      
      SDFJointHolder parentJoint = parentLinkHolder.getJoint();
      if (parentJoint != null)
      {
         rotationTransform.setRotation(parentJoint.getLinkRotation());
         parentLinkToParentJoint = parentJoint.getTransformFromChildLink();
      }
      else
      {
         parentLinkToParentJoint = new RigidBodyTransform();
      }

      RigidBodyTransform modelToParentJoint = new RigidBodyTransform();
      RigidBodyTransform modelToChildJoint = new RigidBodyTransform();

      modelToParentJoint.multiply(modelToParentLink, parentLinkToParentJoint);
      
      modelToChildLink.getRotation(linkRotation);
      
      modelToChildJoint.multiply(modelToChildLink, transformFromChildLink);

      RigidBodyTransform parentJointToModel = new RigidBodyTransform();
      parentJointToModel.invert(modelToParentJoint);

      RigidBodyTransform parentJointToChildJoint = new RigidBodyTransform();
      parentJointToChildJoint.multiply(parentJointToModel, modelToChildJoint);

      transformToParentJoint = parentJointToChildJoint;
      
      parentJointToChildJoint.getTranslation(offsetFromParentJoint);
      rotationTransform.transform(offsetFromParentJoint);
      
      linkRotation.transform(axisInModelFrame, axisInParentFrame);
      
      RigidBodyTransform transformFromParentJoint = new RigidBodyTransform(modelToChildJoint);
      transformFromParentJoint.transform(axisInParentFrame, axisInJointFrame);
      
   }

   public String getName()
   {
      return name;
   }

   public SDFJointType getType()
   {
      return type;
   }

   public Vector3d getAxisInModelFrame()
   {
      return axisInModelFrame;
   }

   public double getUpperLimit()
   {
      return upperLimit;
   }

   public double getLowerLimit()
   {
      return lowerLimit;
   }
   
   public boolean hasLimits()
   {
      return hasLimits;
   }

   public RigidBodyTransform getTransformFromChildLink()
   {
      return transformFromChildLink;
   }

   public SDFLinkHolder getParentLinkHolder()
   {
      return parentLinkHolder;
   }

   public SDFLinkHolder getChildLinkHolder()
   {
      return childLinkHolder;
   }

   public RigidBodyTransform getTransformToParentJoint()
   {
      return transformToParentJoint;
   }

   public double getContactKp()
   {
      return contactKp;
   }

   public double getContactKd()
   {
      return contactKd;
   }

   public double getMaxVel()
   {
      return maxVel;
   }

   public String toString()
   {
      return name;
   }

   public double getDamping()
   {
      return damping;
   }

   public double getFriction()
   {
      return friction;
   }

   public double getEffortLimit()
   {
      return effortLimit;
   }

   public double getVelocityLimit()
   {
      return velocityLimit;
   }
   
   public Matrix3d getLinkRotation()
   {
      return linkRotation;
   }
   
   public Vector3d getOffsetFromParentJoint()
   {
      return offsetFromParentJoint;
   }
   
   public Vector3d getAxisInParentFrame()
   {
      return axisInParentFrame;
   }
   
   public Vector3d getAxisInJointFrame()
   {
      return axisInJointFrame;
   }

   
   // Temporary hack to get force sensors nicely in the code
   public ArrayList<SDFForceSensor> getForceSensors()
   {
      return forceSensors;
   }
   
   public void addForceSensor(SDFForceSensor forceSensor)
   {
      forceSensors.add(forceSensor);
   }
   
   public ArrayList<SDFContactSensor> getContactSensors()
   {
      return contactSensors;
   }
   
   public void addContactSensor(SDFContactSensor contactSensor)
   {
      contactSensors.add(contactSensor);
   }
}
