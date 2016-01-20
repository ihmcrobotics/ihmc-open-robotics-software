package us.ihmc.valkyrie.parameters;

import static us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties.footChamferX;
import static us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties.footChamferY;
import static us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties.footLength;
import static us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties.footWidth;

import java.io.Console;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.SDFConversionsHelper;
import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.SdfLoader.SDFJointHolder;
import us.ihmc.SdfLoader.SDFLinkHolder;
import us.ihmc.SdfLoader.xmlDescription.Collision;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CVXMomentumOptimizerWithGRFPenalizedSmootherNative;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class ValkyrieContactPointParameters extends RobotContactPointParameters
{
   private final ContactableBodiesFactory contactableBodiesFactory = new ContactableBodiesFactory();

   private final List<ImmutablePair<String, Vector3d>> jointNameGroundContactPointMap = new ArrayList<ImmutablePair<String, Vector3d>>();
   private final SideDependentList<ArrayList<Point2d>> footGroundContactPoints = new SideDependentList<>();
   private final SideDependentList<List<Point2d>> handContactPoints = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> handContactPointTransforms = new SideDependentList<>();
   private final SideDependentList<List<Point2d>> thighContactPoints = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> thighContactPointTransforms = new SideDependentList<>();
   private final List<Point2d> chestBackContactPoints = new ArrayList<Point2d>();
   private final RigidBodyTransform chestBackContactPointTransform = new RigidBodyTransform();
   private final List<Point2d> pelvisBackContactPoints = new ArrayList<Point2d>();
   private final RigidBodyTransform pelvisBackContactPointTransform = new RigidBodyTransform();
   private final List<Point2d> pelvisContactPoints = new ArrayList<Point2d>();
   private final RigidBodyTransform pelvisContactPointTransform = new RigidBodyTransform();

   private final boolean CHAMFER_FEET_CORNERS = CVXMomentumOptimizerWithGRFPenalizedSmootherNative.ALLOW_EIGHT_POINTS_AND_ONLY_TWO_PLANES;
   private final DRCRobotJointMap jointMap;
   
   private boolean useSoftGroundContactParameters = false;
   
   public ValkyrieContactPointParameters(DRCRobotJointMap jointMap)
   {
	  this.jointMap = jointMap;
      for (RobotSide robotSide : RobotSide.values)
      {
         footGroundContactPoints.put(robotSide, new ArrayList<Point2d>());
         RigidBodyTransform ankleToSoleFrame = ValkyriePhysicalProperties.getAnkleToSoleFrameTransform(robotSide);

         ArrayList<ImmutablePair<String, Point2d>> footGCs = new ArrayList<>();
         String jointBeforeFootName = jointMap.getJointBeforeFootName(robotSide);
         
         if (CHAMFER_FEET_CORNERS)
         {
        	 footGCs.add(new ImmutablePair<String, Point2d>(jointBeforeFootName, new Point2d(footLength / 2.0 - footChamferX, -footWidth / 2.0)));
        	 footGCs.add(new ImmutablePair<String, Point2d>(jointBeforeFootName, new Point2d(footLength / 2.0 - footChamferX, footWidth / 2.0)));
        	 footGCs.add(new ImmutablePair<String, Point2d>(jointBeforeFootName, new Point2d(-footLength / 2.0 + footChamferX, -footWidth / 2.0)));
        	 footGCs.add(new ImmutablePair<String, Point2d>(jointBeforeFootName, new Point2d(-footLength / 2.0 + footChamferX, footWidth / 2.0)));

        	 footGCs.add(new ImmutablePair<String, Point2d>(jointBeforeFootName, new Point2d(footLength / 2.0, -footWidth / 2.0  + footChamferY)));
        	 footGCs.add(new ImmutablePair<String, Point2d>(jointBeforeFootName, new Point2d(footLength / 2.0, footWidth / 2.0  - footChamferY)));
        	 footGCs.add(new ImmutablePair<String, Point2d>(jointBeforeFootName, new Point2d(-footLength / 2.0, -footWidth / 2.0  + footChamferY)));
        	 footGCs.add(new ImmutablePair<String, Point2d>(jointBeforeFootName, new Point2d(-footLength / 2.0, footWidth / 2.0  - footChamferY)));
         }
         else
         {
        	 footGCs.add(new ImmutablePair<String, Point2d>(jointBeforeFootName, new Point2d(footLength / 2.0, -footWidth / 2.0)));
             footGCs.add(new ImmutablePair<String, Point2d>(jointBeforeFootName, new Point2d(footLength / 2.0, footWidth / 2.0)));
             footGCs.add(new ImmutablePair<String, Point2d>(jointBeforeFootName, new Point2d(-footLength / 2.0, -footWidth / 2.0)));
             footGCs.add(new ImmutablePair<String, Point2d>(jointBeforeFootName, new Point2d(-footLength / 2.0, footWidth / 2.0)));
         }
         
         for (ImmutablePair<String, Point2d> footGC : footGCs)
         {
            footGroundContactPoints.get(robotSide).add(footGC.getRight());

            Point3d gcOffset = new Point3d(footGC.getRight().getX(), footGC.getRight().getY(), 0.0);
            ankleToSoleFrame.transform(gcOffset);
            jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(footGC.getLeft(), new Vector3d(gcOffset)));
         }
      }
      
      setupContactableBodiesFactory(jointMap);
   }
   
   
   private void checkJointChildren(SDFJointHolder joint)
   {
	   SDFLinkHolder link = joint.getChildLinkHolder();
	   for(Collision collision : link.getCollisions())
	   {		   
		   if(collision.getName().contains("_heel") || collision.getName().contains("_toe") || collision.getName().contains("sim_contact") || (collision.getGeometry().getSphere()!=null && Double.parseDouble(collision.getGeometry().getSphere().getRadius())==0.0) )
		   {
			   System.out.println("Simulation contact '" + collision.getName()+"'");
			   Vector3d gcOffset = new Vector3d();;
			   SDFConversionsHelper.poseToTransform(collision.getPose()).get(gcOffset);
			   link.getTransformFromModelReferenceFrame().transform (gcOffset);
			   jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(joint.getName(), gcOffset));
		   }
		   
		   if(collision.getName().contains("ctrl_contact"))
		   {
			   System.out.println("Controller contact '" + collision.getName()+"'");
			   Vector3d gcOffset = new Vector3d();;
			   SDFConversionsHelper.poseToTransform(collision.getPose()).get(gcOffset);
			   link.getTransformFromModelReferenceFrame().transform (gcOffset);
			   boolean assigned = false;
			   
			   for (RobotSide robotSide : RobotSide.values)
			   {
				   if(joint.getName().equals(jointMap.getJointBeforeFootName(robotSide)))
				   {
					   footGroundContactPoints.get(robotSide).add(projectOnPlane(ValkyriePhysicalProperties.getAnkleToSoleFrameTransform(robotSide), gcOffset));
					   assigned = true;
					   break;
				   }
				   else if(joint.getName().equals(jointMap.getJointBeforeHandName(robotSide)))
				   {
					   System.err.println("Hand contacts are not supported ("+collision.getName()+")");
					   assigned = true;
					   break;
				   }
				   else if(joint.getName().equals(jointMap.getChestName()))
				   {
					   System.err.println("Chest contacts are not supported ("+collision.getName()+")");
					   assigned = true;
					   break;
				   }
				   else if(joint.getName().equals(jointMap.getPelvisName()))
				   {
					   System.err.println("Pelvis contacts are not supported ("+collision.getName()+")");
					   // Pelvis back has to be disnguished here
					   assigned = true;
					   break;
				   }
				   else if(joint.getName().equals(jointMap.getNameOfJointBeforeThighs().get(robotSide)))
				   {
					   System.err.println("Thigh contacts are not supported ("+collision.getName()+")");
					   assigned = true;
					   break;
				   }				   
			   }
			   if(!assigned)
			   {
			     System.err.println("Contacts with '" + joint.getName() + "' are not supported ("+collision.getName()+")");
			   }
		   }
	   }
	   
	   for(SDFJointHolder j : link.getChildren())
	   {
		   checkJointChildren(j);
	   }
   }
   
   private Point2d projectOnPlane(RigidBodyTransform plane, Vector3d point)
   {
	   RigidBodyTransform planeInv = new RigidBodyTransform(plane);
	   planeInv.invert();
	   planeInv.transform(point);
	   return new Point2d(point.x,point.y); 
   }
   
   public void setupContactPointsFromRobotModel(DRCRobotModel robot, boolean removeExistingContacts)
   {
	   if(removeExistingContacts) jointNameGroundContactPointMap.clear();
	   GeneralizedSDFRobotModel sdf = robot.getGeneralizedRobotModel();
	   for (SDFLinkHolder link : sdf.getRootLinks())
	   {
		   for (SDFJointHolder joint : link.getChildren())
		   {
			   checkJointChildren(joint);
		   }
	   }
	   setupContactableBodiesFactory(jointMap);
   }
   
   public void addMoreFootContactPointsSimOnly()
   {
      int nContactPointsX = 8;
      int nContactPointsY = 3;

      double dx =  footLength / (nContactPointsX - 1.0);
      double xOffset = footLength / 2.0;

      for (RobotSide robotSide : RobotSide.values)
      {
         for (int ix = 1; ix <= nContactPointsX; ix++)
         {
            double alpha = (ix - 1.0) / (nContactPointsX - 1.0);
            double footWidthAtCurrentX = footWidth;
            if (CHAMFER_FEET_CORNERS)
            {
            	footWidthAtCurrentX -= alpha * footChamferY;
            }
            double dy = footWidthAtCurrentX / (nContactPointsY - 1.0);
            double yOffset = footWidthAtCurrentX / 2.0;

            for (int iy = 1; iy <= nContactPointsY; iy++)
            {
               if ((ix == 1 || ix == nContactPointsX) && (iy == 1 || iy == nContactPointsY)) // Avoid adding corners a second time
                  continue;
               double x = (ix - 1) * dx - xOffset;
               double y = (iy - 1) * dy - yOffset;
               Point3d gcOffset = new Point3d(x, y, 0);

               //footGroundContactPoints.get(robotSide).add( new Point2d(x,y) );
               ValkyriePhysicalProperties.soleToAnkleFrameTransforms.get(robotSide).transform(gcOffset);
               jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(jointMap.getJointBeforeFootName(robotSide), new Vector3d(gcOffset))); // to SCS
            }
         }
      }
      useSoftGroundContactParameters = true;
   }

   private void setupContactableBodiesFactory(DRCRobotJointMap jointMap)
   {
	  if(footGroundContactPoints.size()>0) contactableBodiesFactory.addFootContactParameters(footGroundContactPoints);
      if(chestBackContactPoints.size()>0) contactableBodiesFactory.addChestBackContactParameters(chestBackContactPoints, chestBackContactPointTransform);
      if(handContactPoints.size()>0) contactableBodiesFactory.addHandContactParameters(jointMap.getNameOfJointBeforeHands(), handContactPoints, handContactPointTransforms);
      if(pelvisBackContactPoints.size()>0) contactableBodiesFactory.addPelvisBackContactParameters(pelvisBackContactPoints, pelvisBackContactPointTransform);
      if(pelvisContactPoints.size()>0) contactableBodiesFactory.addPelvisContactParameters(pelvisContactPoints, pelvisContactPointTransform);
      if(thighContactPoints.size()>0) contactableBodiesFactory.addThighContactParameters(jointMap.getNameOfJointBeforeThighs(), thighContactPoints, thighContactPointTransforms);
   }

   @Override
   public RigidBodyTransform getPelvisContactPointTransform()
   {
      return pelvisContactPointTransform;
   }

   @Override
   public List<Point2d> getPelvisContactPoints()
   {
      return pelvisContactPoints;
   }

   @Override
   public RigidBodyTransform getPelvisBackContactPointTransform()
   {
      return pelvisBackContactPointTransform;
   }

   @Override
   public List<Point2d> getPelvisBackContactPoints()
   {
      return pelvisBackContactPoints;
   }

   @Override
   public RigidBodyTransform getChestBackContactPointTransform()
   {
      return chestBackContactPointTransform;
   }

   @Override
   public List<Point2d> getChestBackContactPoints()
   {
      return chestBackContactPoints;
   }

   @Override
   public SideDependentList<RigidBodyTransform> getThighContactPointTransforms()
   {
      return thighContactPointTransforms;
   }

   @Override
   public SideDependentList<List<Point2d>> getThighContactPoints()
   {
      return thighContactPoints;
   }

   @Override
   public List<ImmutablePair<String, Vector3d>> getJointNameGroundContactPointMap()
   {
      return jointNameGroundContactPointMap;
   }

   @Override
   public SideDependentList<ArrayList<Point2d>> getFootContactPoints()
   {
      return footGroundContactPoints;
   }

   @Override
   public ContactableBodiesFactory getContactableBodiesFactory()
   {
      return contactableBodiesFactory;
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

   @Override
   public void setupGroundContactModelParameters(LinearGroundContactModel linearGroundContactModel)
   {
      if (useSoftGroundContactParameters)
      {
         linearGroundContactModel.setZStiffness(4000.0);
         linearGroundContactModel.setZDamping(750.0);
         linearGroundContactModel.setXYStiffness(50000.0);
         linearGroundContactModel.setXYDamping(1000.0);
      }
      else
      {
	     linearGroundContactModel.setZStiffness(2000.0);      
	     linearGroundContactModel.setZDamping(1500.0);      
	     linearGroundContactModel.setXYStiffness(50000.0);      
	     linearGroundContactModel.setXYDamping(2000.0);
      }
   }
}
