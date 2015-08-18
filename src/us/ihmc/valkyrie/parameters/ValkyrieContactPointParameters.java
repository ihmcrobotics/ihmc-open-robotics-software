package us.ihmc.valkyrie.parameters;

import static us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties.footChamferX;
import static us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties.footChamferY;
import static us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties.footLength;
import static us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties.footWidth;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CVXMomentumOptimizerWithGRFPenalizedSmootherNative;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class ValkyrieContactPointParameters extends RobotContactPointParameters
{
   private final ContactableBodiesFactory contactableBodiesFactory = new ContactableBodiesFactory();

   private final List<ImmutablePair<String, Vector3d>> jointNameGroundContactPointMap = new ArrayList<ImmutablePair<String, Vector3d>>();
   private final SideDependentList<ArrayList<Point2d>> footGroundContactPoints = new SideDependentList<>();

   private final boolean CHAMFER_FEET_CORNERS = CVXMomentumOptimizerWithGRFPenalizedSmootherNative.ALLOW_EIGHT_POINTS_AND_ONLY_TWO_PLANES;
   
   public ValkyrieContactPointParameters(DRCRobotJointMap jointMap)
   {
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

   private void setupContactableBodiesFactory(DRCRobotJointMap jointMap)
   {
      contactableBodiesFactory.addFootContactParameters(getFootContactPoints());
   }

   @Override
   public RigidBodyTransform getPelvisContactPointTransform()
   {
      return null;
   }

   @Override
   public List<Point2d> getPelvisContactPoints()
   {
      return null;
   }

   @Override
   public RigidBodyTransform getPelvisBackContactPointTransform()
   {
      return null;
   }

   @Override
   public List<Point2d> getPelvisBackContactPoints()
   {
      return null;
   }

   @Override
   public RigidBodyTransform getChestBackContactPointTransform()
   {
      return null;
   }

   @Override
   public List<Point2d> getChestBackContactPoints()
   {
      return null;
   }

   @Override
   public SideDependentList<RigidBodyTransform> getThighContactPointTransforms()
   {
      return null;
   }

   @Override
   public SideDependentList<List<Point2d>> getThighContactPoints()
   {
      return null;
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
      return null;
   }

   @Override
   public SideDependentList<List<Point2d>> getHandContactPoints()
   {
      return null;
   }

   @Override
   public void setupGroundContactModelParameters(LinearGroundContactModel linearGroundContactModel)
   {
      linearGroundContactModel.setZStiffness(2000.0);      
      linearGroundContactModel.setZDamping(1500.0);      
      linearGroundContactModel.setXYStiffness(50000.0);      
      linearGroundContactModel.setXYDamping(2000.0);      
   }
}
