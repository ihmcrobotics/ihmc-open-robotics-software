package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.BoxCollisionMeshDefinitionData;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CollisionMeshDefinitionData;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CollisionMeshDefinitionDataHolder;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CylinderCollisionMeshDefinitionData;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.SphereCollisionMeshDefinitionData;

public class RobotDescription implements RobotDescriptionNode, GraphicsObjectsHolder
{
   private String name;
   private final ArrayList<JointDescription> rootJoints = new ArrayList<>();

   public RobotDescription(String name)
   {
      this.setName(name);
   }

   public void addRootJoint(JointDescription rootJoint)
   {
      this.rootJoints.add(rootJoint);
   }

   @Override
   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public ArrayList<JointDescription> getRootJoints()
   {
      return rootJoints;
   }

   @Override
   public ArrayList<JointDescription> getChildrenJoints()
   {
      return getRootJoints();
   }

   public JointDescription getJointDescription(String name)
   {
      for (JointDescription rootJoint : rootJoints)
      {
         JointDescription jointDescription = getJointDescriptionRecursively(name, rootJoint);
         if (jointDescription != null)
            return jointDescription;
      }

      return null;
   }

   private JointDescription getJointDescriptionRecursively(String name, JointDescription jointDescription)
   {
      if (jointDescription.getName().equals(name))
         return jointDescription;

      ArrayList<JointDescription> childJointDescriptions = jointDescription.getChildrenJoints();
      for (JointDescription childJointDescription : childJointDescriptions)
      {
         JointDescription jointDescriptionRecursively = getJointDescriptionRecursively(name, childJointDescription);
         if (jointDescriptionRecursively != null)
            return jointDescriptionRecursively;
      }
      return null;
   }

   @Override
   public ArrayList<CollisionMeshDescription> getCollisionObjects(String name)
   {
      JointDescription jointDescription = getJointDescription(name);
      if (jointDescription == null)
         return null;

      return jointDescription.getLink().getCollisionMeshes();
   }

   @Override
   public Graphics3DObject getGraphicsObject(String name)
   {
      JointDescription jointDescription = getJointDescription(name);
      if (jointDescription == null)
         return null;

      return jointDescription.getLink().getLinkGraphics();
   }

   public LinkDescription getLinkDescription(String name)
   {
      JointDescription jointDescription = getJointDescription(name);
      if (jointDescription == null)
         return null;

      return jointDescription.getLink();
   }

   @Override
   public void scale(double factor, double massScalePower, List<String> ignoreInertiaScaleJointList)
   {
      JointDescription.scaleChildrenJoint(getChildrenJoints(), factor, massScalePower, ignoreInertiaScaleJointList);
   }

   public void addCollisionMeshDefinitionData(CollisionMeshDefinitionDataHolder collisionMeshDefinitionDataHolder)
   {
      boolean isVisible = collisionMeshDefinitionDataHolder.isVisible();
      List<CollisionMeshDefinitionData> collisionMeshDefinitionDataList = collisionMeshDefinitionDataHolder.getCollisionMeshDefinitionData();
      int numberOfDefinitionData = collisionMeshDefinitionDataList.size();

      for (int i = 0; i < numberOfDefinitionData; i++)
      {
         if (collisionMeshDefinitionDataList.get(i) instanceof SphereCollisionMeshDefinitionData)
         {
            addSphereCollisionMeshDefinitionData((SphereCollisionMeshDefinitionData) collisionMeshDefinitionDataList.get(i), isVisible);
         }
         else if (collisionMeshDefinitionDataList.get(i) instanceof CylinderCollisionMeshDefinitionData)
         {
            addCylinderCollisionMeshDefinitionData((CylinderCollisionMeshDefinitionData) collisionMeshDefinitionDataList.get(i), isVisible);
         }
         else if (collisionMeshDefinitionDataList.get(i) instanceof BoxCollisionMeshDefinitionData)
         {
            addBoxCollisionMeshDefinitionData((BoxCollisionMeshDefinitionData) collisionMeshDefinitionDataList.get(i), isVisible);
         }
         else
         {
            throw new IllegalArgumentException("The type of " + getName() + " is not matched among the simple shape Box3D, Sphere3D, Cylinder3D, Capsule3D");
         }
      }
   }

   private void addBoxCollisionMeshDefinitionData(BoxCollisionMeshDefinitionData collisionMeshDefinitionData, boolean isVisible)
   {
      LinkDescription linkDescription = getLinkDescription(collisionMeshDefinitionData.getParentJointName());

      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.identity();
      collisionMesh.transform(collisionMeshDefinitionData.getTransformToParentJoint());
      collisionMesh.addCubeReferencedAtCenter(collisionMeshDefinitionData.getLength(), collisionMeshDefinitionData.getWidth(),
                                              collisionMeshDefinitionData.getHeight());
      collisionMesh.setCollisionGroup(collisionMeshDefinitionData.getCollisionGroup());
      collisionMesh.setCollisionMask(collisionMeshDefinitionData.getCollisionMask());
      linkDescription.addCollisionMesh(collisionMesh);

      if (isVisible)
      {
         LinkGraphicsDescription linkGraphics;
         if (linkDescription.getLinkGraphics() != null)
            linkGraphics = linkDescription.getLinkGraphics();
         else
         {
            linkGraphics = new LinkGraphicsDescription();
            linkDescription.setLinkGraphics(linkGraphics);
         }

         linkGraphics.identity();
         linkGraphics.transform(collisionMeshDefinitionData.getTransformToParentJoint());
         linkGraphics.translate(0, 0, -0.5 * collisionMeshDefinitionData.getHeight());
         linkGraphics.addCube(collisionMeshDefinitionData.getLength(), collisionMeshDefinitionData.getWidth(), collisionMeshDefinitionData.getHeight(),
                              collisionMeshDefinitionData.getYoAppearance());
      }
   }

   private void addSphereCollisionMeshDefinitionData(SphereCollisionMeshDefinitionData collisionMeshDefinitionData, boolean isVisible)
   {
      LinkDescription linkDescription = getLinkDescription(collisionMeshDefinitionData.getParentJointName());

      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.identity();
      collisionMesh.transform(collisionMeshDefinitionData.getTransformToParentJoint());
      collisionMesh.addSphere(collisionMeshDefinitionData.getRadius());
      collisionMesh.setCollisionGroup(collisionMeshDefinitionData.getCollisionGroup());
      collisionMesh.setCollisionMask(collisionMeshDefinitionData.getCollisionMask());
      linkDescription.addCollisionMesh(collisionMesh);

      if (isVisible)
      {
         LinkGraphicsDescription linkGraphics;
         if (linkDescription.getLinkGraphics() != null)
            linkGraphics = linkDescription.getLinkGraphics();
         else
         {
            linkGraphics = new LinkGraphicsDescription();
            linkDescription.setLinkGraphics(linkGraphics);
         }

         linkGraphics.identity();
         linkGraphics.transform(collisionMeshDefinitionData.getTransformToParentJoint());
         linkGraphics.addSphere(collisionMeshDefinitionData.getRadius(), collisionMeshDefinitionData.getYoAppearance());

      }
   }

   private void addCylinderCollisionMeshDefinitionData(CylinderCollisionMeshDefinitionData collisionMeshDefinitionData, boolean isVisible)
   {
      LinkDescription linkDescription = getLinkDescription(collisionMeshDefinitionData.getParentJointName());

      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.identity();
      collisionMesh.transform(collisionMeshDefinitionData.getTransformToParentJoint());
      collisionMesh.addCylinderReferencedAtBottomMiddle(collisionMeshDefinitionData.getRadius(), collisionMeshDefinitionData.getHeight());
      collisionMesh.setCollisionGroup(collisionMeshDefinitionData.getCollisionGroup());
      collisionMesh.setCollisionMask(collisionMeshDefinitionData.getCollisionMask());
      linkDescription.addCollisionMesh(collisionMesh);

      if (isVisible)
      {
         LinkGraphicsDescription linkGraphics;
         if (linkDescription.getLinkGraphics() != null)
            linkGraphics = linkDescription.getLinkGraphics();
         else
         {
            linkGraphics = new LinkGraphicsDescription();
            linkDescription.setLinkGraphics(linkGraphics);
         }

         linkGraphics.identity();
         linkGraphics.transform(collisionMeshDefinitionData.getTransformToParentJoint());
         linkGraphics.addCylinder(collisionMeshDefinitionData.getHeight(), collisionMeshDefinitionData.getRadius(),
                                  collisionMeshDefinitionData.getYoAppearance());
      }
   }
}
