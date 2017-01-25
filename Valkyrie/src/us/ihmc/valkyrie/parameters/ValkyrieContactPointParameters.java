package us.ihmc.valkyrie.parameters;

import static us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties.footLength;
import static us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties.footWidth;
import static us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties.soleToAnkleFrameTransforms;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.SDFConversionsHelper;
import us.ihmc.modelFileLoaders.SdfLoader.SDFJointHolder;
import us.ihmc.modelFileLoaders.SdfLoader.SDFLinkHolder;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.Collision;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry.Sphere;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class ValkyrieContactPointParameters extends RobotContactPointParameters
{
   private final SideDependentList<ArrayList<Point2d>> footGroundContactPoints = new SideDependentList<>();
   private final SideDependentList<List<Point2d>> handContactPoints = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> handContactPointTransforms = new SideDependentList<>();

   private final DRCRobotJointMap jointMap;

   public ValkyrieContactPointParameters(DRCRobotJointMap jointMap, FootContactPoints footContactPoints)
   {
      super(jointMap, footWidth, footLength, soleToAnkleFrameTransforms);
      this.jointMap = jointMap;

      if (footContactPoints == null)
         createDefaultFootContactPoints();
      else
         createContactPoints(footContactPoints);
   }

   private void checkJointChildren(SDFJointHolder joint)
   {
      SDFLinkHolder link = joint.getChildLinkHolder();
      for (Collision collision : link.getCollisions())
      {
         String name = collision.getName();
         Sphere sphere = collision.getGeometry().getSphere();
         if (name.contains("_heel") || name.contains("_toe") || name.contains("sim_contact") || (sphere != null && Double.parseDouble(sphere.getRadius()) == 0.0))
         {
            System.out.println("Simulation contact '" + name + "'");
            Vector3d gcOffset = new Vector3d();

            SDFConversionsHelper.poseToTransform(collision.getPose()).getTranslation(gcOffset);
            link.getTransformFromModelReferenceFrame().transform(gcOffset);
            addSimulationContactPoint(joint.getName(), gcOffset);
         }

         if (name.contains("ctrl_contact"))
         {
            System.out.println("Controller contact '" + name + "'");
            Vector3d gcOffset = new Vector3d();

            SDFConversionsHelper.poseToTransform(collision.getPose()).getTranslation(gcOffset);
            link.getTransformFromModelReferenceFrame().transform(gcOffset);
            boolean assigned = false;

            for (RobotSide robotSide : RobotSide.values)
            {
               if (joint.getName().equals(jointMap.getJointBeforeFootName(robotSide)))
               {
                  footGroundContactPoints.get(robotSide).add(projectOnPlane(ValkyriePhysicalProperties.getSoleToAnkleFrameTransform(robotSide), gcOffset));
                  assigned = true;
                  break;
               }
               else if (joint.getName().equals(jointMap.getJointBeforeHandName(robotSide)))
               {
                  System.err.println("Hand contacts are not supported (" + name + ")");
                  assigned = true;
                  break;
               }
               else if (joint.getName().equals(jointMap.getChestName()))
               {
                  System.err.println("Chest contacts are not supported (" + name + ")");
                  assigned = true;
                  break;
               }
               else if (joint.getName().equals(jointMap.getPelvisName()))
               {
                  System.err.println("Pelvis contacts are not supported (" + name + ")");
                  // Pelvis back has to be disnguished here
                  assigned = true;
                  break;
               }
               else if (joint.getName().equals(jointMap.getNameOfJointBeforeThighs().get(robotSide)))
               {
                  System.err.println("Thigh contacts are not supported (" + name + ")");
                  assigned = true;
                  break;
               }
            }
            if (!assigned)
            {
               System.err.println("Contacts with '" + joint.getName() + "' are not supported (" + name + ")");
            }
         }
      }

      for (SDFJointHolder child : link.getChildren())
      {
         checkJointChildren(child);
      }
   }

   private Point2d projectOnPlane(RigidBodyTransform plane, Vector3d point)
   {
      RigidBodyTransform planeInv = new RigidBodyTransform(plane);
      planeInv.invert();
      planeInv.transform(point);
      return new Point2d(point.getX(), point.getY());
   }

   public void setupContactPointsFromRobotModel(GeneralizedSDFRobotModel sdf, boolean removeExistingContacts)
   {
      if (removeExistingContacts)
         clearSimulationContactPoints();

      for (SDFLinkHolder link : sdf.getRootLinks())
      {
         for (SDFJointHolder joint : link.getChildren())
         {
            checkJointChildren(joint);
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         if (!footGroundContactPoints.get(robotSide).isEmpty())
         {
            clearControllerFootContactPoints();
            setControllerFootContactPoint(robotSide, footGroundContactPoints.get(robotSide));
         }
      }
   }

   @Override
   public SideDependentList<RigidBodyTransform> getHandContactPointTransforms()
   {
      return handContactPointTransforms;
   }

   @Override
   public SideDependentList<List<Point2d>> getHandContactPoints()
   {
      return handContactPoints;
   }
}
