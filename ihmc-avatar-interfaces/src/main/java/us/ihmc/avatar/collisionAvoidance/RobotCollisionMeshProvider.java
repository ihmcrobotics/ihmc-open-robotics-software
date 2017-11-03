package us.ihmc.avatar.collisionAvoidance;

import java.util.ArrayList;

import gnu.trove.map.hash.THashMap;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.geometry.polytope.ConvexPolytopeConstructor;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotDescription.CapsuleDescriptionReadOnly;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.ConvexPolytopeDescriptionReadOnly;
import us.ihmc.robotics.robotDescription.ConvexShapeDescription;
import us.ihmc.robotics.robotDescription.CubeDescriptionReadOnly;
import us.ihmc.robotics.robotDescription.CylinderDescriptionReadOnly;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.SphereDescriptionReadOnly;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class RobotCollisionMeshProvider
{
   private final int defaultCurvedSurfaceDivisions;

   public RobotCollisionMeshProvider(int numberOfCurvedSurfaceDivisions)
   {
      this.defaultCurvedSurfaceDivisions = numberOfCurvedSurfaceDivisions;
   }

   public THashMap<RigidBody, FrameConvexPolytope> createCollisionMeshesFromRobotDescription(FullRobotModel fullRobotModel, RobotDescription robotDescription)
   {
      ArrayList<JointDescription> rootJointDescriptions = robotDescription.getRootJoints();
      if (rootJointDescriptions.size() > 1 || !(rootJointDescriptions.get(0) instanceof FloatingJointDescription))
         throw new RuntimeException("There should be only one floating joint");
      FloatingJointDescription rootJointDescription = (FloatingJointDescription) rootJointDescriptions.get(0);
      THashMap<RigidBody, FrameConvexPolytope> collisionMeshMap = new THashMap<>();
      recursivelyAddCollisionMeshes(collisionMeshMap, rootJointDescription, fullRobotModel);
      return collisionMeshMap;
   }
   
   private void recursivelyAddCollisionMeshes(THashMap<RigidBody, FrameConvexPolytope> collisionMeshMap, JointDescription jointDescription, FullRobotModel fullRobotModel)
   {
      if(!(jointDescription.getName() == fullRobotModel.getRootJoint().getName()))
      {
         LinkDescription linkDescription = jointDescription.getLink();
         InverseDynamicsJoint joint = fullRobotModel.getOneDoFJointByName(jointDescription.getName());
         RigidBody rigidBody = joint.getSuccessor();
         PrintTools.debug("Link : " + linkDescription.getName() +  " Joint: " + joint.getName());
         collisionMeshMap.put(rigidBody, createCollisionMesh(rigidBody, linkDescription));
      }
      for (JointDescription childJointDescription : jointDescription.getChildrenJoints())
      {
         recursivelyAddCollisionMeshes(collisionMeshMap, childJointDescription, fullRobotModel);
      }
   }
   
   private final Vector3D centerOfMassOffset = new Vector3D();
   
   public FrameConvexPolytope createCollisionMesh(RigidBody rigidBody, ArrayList<CollisionMeshDescription> meshDescriptions)
   {
      centerOfMassOffset.setToZero();
      return ConvexPolytopeConstructor.createFramePolytope(rigidBody.getBodyFixedFrame(), getCollisionMeshPoints(meshDescriptions, centerOfMassOffset));
   }
   
   public FrameConvexPolytope createCollisionMesh(RigidBody rigidBody, LinkDescription linkDescriptions)
   {
      linkDescriptions.getCenterOfMassOffset(centerOfMassOffset);
      return ConvexPolytopeConstructor.createFramePolytope(rigidBody.getBodyFixedFrame(), getCollisionMeshPoints(linkDescriptions.getCollisionMeshes(), centerOfMassOffset));
   }

   public ArrayList<Point3D> getCollisionMeshPoints(ArrayList<CollisionMeshDescription> meshDescriptions, Vector3D centerOfMassOffset)
   {
      ArrayList<ConvexShapeDescription> collisionShapeDescriptions = new ArrayList<>();
      for(int i = 0; i < meshDescriptions.size(); i++)
         meshDescriptions.get(i).getConvexShapeDescriptions(collisionShapeDescriptions);
      ArrayList<Point3D> pointsForRigidBody = new ArrayList<>();
      for (ConvexShapeDescription shapeDescription : collisionShapeDescriptions)
      {
         ArrayList<Point3D> pointsForShapeDescription = new ArrayList<>();
         if (shapeDescription instanceof SphereDescriptionReadOnly)
         {
            PrintTools.debug("Adding spherical mesh");
            RigidBodyTransform transform = new RigidBodyTransform();
            ((SphereDescriptionReadOnly) shapeDescription).getRigidBodyTransform(transform);
            ConvexPolytopeConstructor.getCollisionMeshPointsForSphere(transform, ((SphereDescriptionReadOnly) shapeDescription).getRadius(), defaultCurvedSurfaceDivisions, pointsForShapeDescription);
         }
         else if (shapeDescription instanceof CapsuleDescriptionReadOnly)
         {
            PrintTools.debug("Adding capsule mesh");
            LineSegment3D lineSegment = new LineSegment3D();
            ((CapsuleDescriptionReadOnly) shapeDescription).getCapToCapLineSegment(lineSegment);
            ConvexPolytopeConstructor.getCollisionMeshPointsForCapsule(lineSegment, ((CapsuleDescriptionReadOnly) shapeDescription).getRadius(), defaultCurvedSurfaceDivisions, pointsForShapeDescription);
         }
         else if (shapeDescription instanceof CylinderDescriptionReadOnly)
         {
            PrintTools.debug("Adding cylinderical mesh");
            RigidBodyTransform transform = new RigidBodyTransform();
            ((CylinderDescriptionReadOnly) shapeDescription).getRigidBodyTransformToCenter(transform);
            ConvexPolytopeConstructor.getCylindericalCollisionMesh(transform, ((CylinderDescriptionReadOnly) shapeDescription).getRadius(), ((CylinderDescriptionReadOnly) shapeDescription).getHeight(), defaultCurvedSurfaceDivisions, pointsForShapeDescription);
         }
         else if (shapeDescription instanceof CubeDescriptionReadOnly)
         {
            PrintTools.debug("Adding cube mesh");
            RigidBodyTransform transform = new RigidBodyTransform();
            ((CubeDescriptionReadOnly) shapeDescription).getRigidBodyTransformToCenter(transform);
            ConvexPolytopeConstructor.getCuboidCollisionMesh(transform, ((CubeDescriptionReadOnly) shapeDescription).getLengthX(), ((CubeDescriptionReadOnly) shapeDescription).getWidthY(), ((CubeDescriptionReadOnly) shapeDescription).getHeightZ(), pointsForShapeDescription);
         }
         else if (shapeDescription instanceof ConvexPolytopeDescriptionReadOnly)
         {
            PrintTools.debug("Adding arbitrary mesh");
            ((ConvexPolytopeDescriptionReadOnly) shapeDescription).getConvexPolytope().getVertices(pointsForShapeDescription);
         }
         else
            throw new RuntimeException("Unhandled collision mesh description shape: " + shapeDescription.getClass());
         pointsForRigidBody.addAll(pointsForShapeDescription);
      }
      centerOfMassOffset.negate();
      ConvexPolytopeConstructor.shiftCentroid(centerOfMassOffset, pointsForRigidBody);
      return pointsForRigidBody;
   }
}
