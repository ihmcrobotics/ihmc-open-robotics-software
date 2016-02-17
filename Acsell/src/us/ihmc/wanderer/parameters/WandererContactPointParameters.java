package us.ihmc.wanderer.parameters;

import static us.ihmc.wanderer.parameters.WandererPhysicalProperties.footLength;
import static us.ihmc.wanderer.parameters.WandererPhysicalProperties.footWidth;
import static us.ihmc.wanderer.parameters.WandererPhysicalProperties.toeWidth;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class WandererContactPointParameters extends RobotContactPointParameters
{
   private final ContactableBodiesFactory contactableBodiesFactory = new ContactableBodiesFactory();

   private final List<ImmutablePair<String, Vector3d>> jointNameGroundContactPointMap = new ArrayList<ImmutablePair<String, Vector3d>>();
   private final SideDependentList<ArrayList<Point2d>> footGroundContactPoints = new SideDependentList<ArrayList<Point2d>>();

   public WandererContactPointParameters(DRCRobotJointMap jointMap)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footGroundContactPoints.put(robotSide, new ArrayList<Point2d>());
         RigidBodyTransform ankleToSoleFrame = WandererPhysicalProperties.getAnkleToSoleFrameTransform(robotSide);

         ArrayList<ImmutablePair<String, Point2d>> footGCs = new ArrayList<ImmutablePair<String, Point2d>>();
         String jointBeforeFootName = jointMap.getJointBeforeFootName(robotSide);
         double shrinkFootLength = footLength / 2.0 - 0.005;
         double shrinkToeWidth = toeWidth / 2.0 - 0.01;
         double shrinkHeelWdith = footWidth / 2.0 - 0.01;
         footGCs.add(new ImmutablePair<String, Point2d>(jointBeforeFootName, new Point2d(shrinkFootLength, -shrinkToeWidth)));
         footGCs.add(new ImmutablePair<String, Point2d>(jointBeforeFootName, new Point2d(shrinkFootLength, shrinkToeWidth)));
         footGCs.add(new ImmutablePair<String, Point2d>(jointBeforeFootName, new Point2d(-shrinkFootLength, -shrinkHeelWdith)));
         footGCs.add(new ImmutablePair<String, Point2d>(jointBeforeFootName, new Point2d(-shrinkFootLength, shrinkHeelWdith)));

         //SCS contact points
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
      linearGroundContactModel.setZStiffness(1500.0);      
      linearGroundContactModel.setZDamping(750.0);      
      linearGroundContactModel.setXYStiffness(25000.0);      
      linearGroundContactModel.setXYDamping(750.0);      
   }
}
