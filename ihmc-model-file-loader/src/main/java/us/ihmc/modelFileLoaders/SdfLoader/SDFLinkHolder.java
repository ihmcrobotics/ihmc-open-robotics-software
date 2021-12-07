package us.ihmc.modelFileLoaders.SdfLoader;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.modelFileLoaders.ModelFileLoaderConversionsHelper;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.Collision;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFLink;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFSensor;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFVisual;

public class SDFLinkHolder
{
   // From SDF File
   private String name;
   private final RigidBodyTransform transformToModelReferenceFrame;
   private double mass;
   private final RigidBodyTransform inertialFrameWithRespectToLinkFrame;
   private final Matrix3D inertia;

   private double contactKp = 0.0;
   private double contactKd = 0.0;
   private double contactMaxVel = 0.0;;

   private final List<SDFVisual> visuals;
   private final List<SDFSensor> sensors;
   private final List<Collision> collisions;

   // Set by loader
   private SDFJointHolder joint = null;
   private final ArrayList<SDFJointHolder> childeren = new ArrayList<SDFJointHolder>();


   // Calculated
   private final Vector3D CoMOffset = new Vector3D();

   public SDFLinkHolder(SDFLink sdfLink)
   {
     name = ModelFileLoaderConversionsHelper.sanitizeJointName(sdfLink.getName());
     transformToModelReferenceFrame = ModelFileLoaderConversionsHelper.poseToTransform(sdfLink.getPose());

     if(sdfLink.getInertial() != null)
     {
        inertialFrameWithRespectToLinkFrame = ModelFileLoaderConversionsHelper.poseToTransform(sdfLink.getInertial().getPose());
        mass = Double.parseDouble(sdfLink.getInertial().getMass());
        inertia = ModelFileLoaderConversionsHelper.sdfInertiaToMatrix3d(sdfLink.getInertial().getInertia());
     }
     else
     {
        inertialFrameWithRespectToLinkFrame = new RigidBodyTransform();
        mass = 0.0;
        inertia = new Matrix3D();
     }
     visuals = sdfLink.getVisuals();

     sensors = sdfLink.getSensors();
     if(sdfLink.getCollisions() != null)
     {
        collisions = sdfLink.getCollisions();

        if((sdfLink.getCollisions().get(0) != null) && (sdfLink.getCollisions().get(0).getSurface() != null)
                 && (sdfLink.getCollisions().get(0).getSurface().getContact() != null)
                 && (sdfLink.getCollisions().get(0).getSurface().getContact().getOde() != null))
         {
            if (sdfLink.getCollisions().get(0).getSurface().getContact().getOde().getKp() != null)
               contactKp = Double.parseDouble(sdfLink.getCollisions().get(0).getSurface().getContact().getOde().getKp());
            if (sdfLink.getCollisions().get(0).getSurface().getContact().getOde().getKd() != null)
               contactKd = Double.parseDouble(sdfLink.getCollisions().get(0).getSurface().getContact().getOde().getKd());
            if (sdfLink.getCollisions().get(0).getSurface().getContact().getOde().getMaxVel() != null)
               contactMaxVel = Double.parseDouble(sdfLink.getCollisions().get(0).getSurface().getContact().getOde().getMaxVel());
         }
      }
      else
      {
         collisions = new ArrayList<Collision>();
      }
   }

   public RigidBodyTransform getTransformFromModelReferenceFrame()
   {
      return transformToModelReferenceFrame;
   }

   public void calculateCoMOffset()
   {

      RigidBodyTransform modelFrameToJointFrame = new RigidBodyTransform();
      if(joint != null)
      {
         modelFrameToJointFrame.set(joint.getTransformFromChildLink()); // H_4^3
      }
      RigidBodyTransform jointFrameToModelFrame = new RigidBodyTransform();    // H_3^4
      jointFrameToModelFrame.setAndInvert(modelFrameToJointFrame);
      RigidBodyTransform modelFrameToInertialFrame = inertialFrameWithRespectToLinkFrame;    // H_4^5

      RigidBodyTransform jointFrameToInertialFrame = new RigidBodyTransform();
      jointFrameToInertialFrame.set(jointFrameToModelFrame);
      jointFrameToInertialFrame.multiply(modelFrameToInertialFrame);

      Vector3D CoMOffset = new Vector3D();
      Matrix3D inertialFrameRotation = new Matrix3D();

      jointFrameToInertialFrame.get(inertialFrameRotation, CoMOffset);

      if(!inertialFrameRotation.epsilonEquals(MatrixTools.IDENTITY, 1e-5))
      {
         inertialFrameRotation.transpose();
         inertia.multiply(inertialFrameRotation);
         inertialFrameRotation.transpose();
         inertialFrameRotation.multiply(inertia);

         inertia.set(inertialFrameRotation);
//         inertia.set(InertiaTools.rotate(inertialFrameRotation, inertia));
         inertialFrameWithRespectToLinkFrame.setRotationAndZeroTranslation(MatrixTools.IDENTITY);
      }

      this.CoMOffset.set(CoMOffset);
   }

   public ArrayList<SDFJointHolder> getChildren()
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

   @Override
   public String toString()
   {
      return name;
   }

   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public List<SDFVisual> getVisuals()
   {
     return visuals;
   }

   public double getMass()
   {
      return mass;
   }

   public Matrix3D getInertia()
   {
      return inertia;
   }

   public Vector3D getCoMOffset()
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

   public List<SDFSensor> getSensors()
   {
      return sensors;
   }

   public List<Collision> getCollisions()
   {
      return collisions;
   }

   public void setMass(double mass)
   {
      this.mass = mass;
   }

   public void setInertialFrameWithRespectToLinkFrame(RigidBodyTransform newTransform)
   {
      this.inertialFrameWithRespectToLinkFrame.set(newTransform);
   }

   public RigidBodyTransform getInertialFrameWithRespectToLinkFrame()
   {
      return inertialFrameWithRespectToLinkFrame;
   }

   public void setInertia(Matrix3D inertia)
   {
      this.inertia.set(inertia);
   }
}
