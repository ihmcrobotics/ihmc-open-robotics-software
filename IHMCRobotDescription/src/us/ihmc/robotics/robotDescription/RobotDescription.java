package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphicsDescription.Graphics3DObject;

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
   public CollisionMeshDescription getCollisionObject(String name)
   {
      JointDescription jointDescription = getJointDescription(name);
      if (jointDescription == null)
         return null;

      return jointDescription.getLink().getCollisionMesh();
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
}
