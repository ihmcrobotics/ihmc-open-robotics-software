package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.simulationconstructionset.physics.CollisionShapeWithLink;

public class ContactingExternalForcePoint extends ExternalForcePoint
{

   private static final long serialVersionUID = -5919153372698232766L;

   private final YoFrameVector surfaceNormalInJointFrame;
   private final IntegerYoVariable indexOfContactingPair;
   private final BooleanYoVariable isSlipping;

   private int index = -1;

   //TODO: This isn't rewindable...
   private CollisionShapeWithLink collisionShape;

   public ContactingExternalForcePoint(String name, Joint parentJoint, YoVariableRegistry registry)
   {
      super(name, registry);
      
      this.setParentJoint(parentJoint);
      this.surfaceNormalInJointFrame = new YoFrameVector(name + "SurfaceNormal", null, registry);
      indexOfContactingPair = new IntegerYoVariable(name + "PairIndex", registry);
      isSlipping = new BooleanYoVariable(name + "IsSlipping", registry);
      
      indexOfContactingPair.set(-1);
   }

   public int getIndexOfContactingPair()
   {
      return indexOfContactingPair.getIntegerValue();
   }
   
   public void setIndexOfContactingPair(int indexOfContactingPair)
   {
      this.indexOfContactingPair.set(indexOfContactingPair);
   }

   public void setIsSlipping(boolean isSlipping)
   {
      this.isSlipping.set(isSlipping);
   }

   public boolean getIsSlipping()
   {
      return isSlipping.getBooleanValue();
   }

   public void setIndex(int index)
   {
      this.index = index;
   }
   
   public int getIndex()
   {
      return index;
   }

   public Link getLink()
   {
      return parentJoint.getLink();
   }
   
   private final Vector3D tempSurfaceNormal = new Vector3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   
   public void setSurfaceNormalInWorld(Vector3D surfaceNormalInWorld)
   {
      //TODO: Make more efficient by having getTransformFromWorld() in Joint...
      tempSurfaceNormal.set(surfaceNormalInWorld);

      parentJoint.getTransformToWorld(tempTransform);
      tempTransform.invert();

      tempTransform.transform(tempSurfaceNormal);

      this.surfaceNormalInJointFrame.set(tempSurfaceNormal);
   }
   
   public void getSurfaceNormalInWorld(Vector3D surfaceNormalInWorldToPack)
   {
      this.surfaceNormalInJointFrame.get(surfaceNormalInWorldToPack);
      parentJoint.getTransformToWorld(tempTransform);
      tempTransform.transform(surfaceNormalInWorldToPack);
   }

   public boolean isInContact()
   {
      return (getIndexOfContactingPair() != -1);
   }

   public void setCollisionShape(CollisionShapeWithLink collisionShape)
   {
      this.collisionShape = collisionShape;
      if (collisionShape.getLink() != getLink())
      {
         throw new RuntimeException("Inconsistent links...");
      }
   }

   public CollisionShapeWithLink getCollisionShape()
   {
      return collisionShape;
   }

}
