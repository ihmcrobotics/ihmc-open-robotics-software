package us.ihmc.valkyrie.paramaters;

import static us.ihmc.valkyrie.paramaters.ValkyriePhysicalProperties.footLength;
import static us.ihmc.valkyrie.paramaters.ValkyriePhysicalProperties.footWidth;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.utilities.math.geometry.Transform3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotContactPointParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;

public class ValkyrieContactPointParameters extends DRCRobotContactPointParameters
{
   private final ContactableBodiesFactory contactableBodiesFactory = new ContactableBodiesFactory();

   private final List<Pair<String, Vector3d>> jointNameGroundContactPointMap = new ArrayList<Pair<String, Vector3d>>();
   private final SideDependentList<ArrayList<Point2d>> footGroundContactPoints = new SideDependentList<>();

   public ValkyrieContactPointParameters(DRCRobotJointMap jointMap)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footGroundContactPoints.put(robotSide, new ArrayList<Point2d>());
         Transform3d ankleToSoleFrame = ValkyriePhysicalProperties.getAnkleToSoleFrameTransform(robotSide);

         ArrayList<Pair<String, Point2d>> footGCs = new ArrayList<>();
         String jointBeforeFootName = jointMap.getJointBeforeFootName(robotSide);
         footGCs.add(new Pair<String, Point2d>(jointBeforeFootName, new Point2d(footLength / 2.0, -footWidth / 2.0)));
         footGCs.add(new Pair<String, Point2d>(jointBeforeFootName, new Point2d(footLength / 2.0, footWidth / 2.0)));
         footGCs.add(new Pair<String, Point2d>(jointBeforeFootName, new Point2d(-footLength / 2.0, -footWidth / 2.0)));
         footGCs.add(new Pair<String, Point2d>(jointBeforeFootName, new Point2d(-footLength / 2.0, footWidth / 2.0)));

         for (Pair<String, Point2d> footGC : footGCs)
         {
            footGroundContactPoints.get(robotSide).add(footGC.second());

            Point3d gcOffset = new Point3d(footGC.second().getX(), footGC.second().getY(), 0.0);
            ankleToSoleFrame.transform(gcOffset);
            jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(footGC.first(), new Vector3d(gcOffset)));
         }
      }

      setupContactableBodiesFactory(jointMap);
   }

   private void setupContactableBodiesFactory(DRCRobotJointMap jointMap)
   {
      contactableBodiesFactory.addFootContactParameters(getFootContactPoints());
   }

   @Override
   public Transform3d getPelvisContactPointTransform()
   {
      return null;
   }

   @Override
   public List<Point2d> getPelvisContactPoints()
   {
      return null;
   }

   @Override
   public Transform3d getPelvisBackContactPointTransform()
   {
      return null;
   }

   @Override
   public List<Point2d> getPelvisBackContactPoints()
   {
      return null;
   }

   @Override
   public Transform3d getChestBackContactPointTransform()
   {
      return null;
   }

   @Override
   public List<Point2d> getChestBackContactPoints()
   {
      return null;
   }

   @Override
   public SideDependentList<Transform3d> getThighContactPointTransforms()
   {
      return null;
   }

   @Override
   public SideDependentList<List<Point2d>> getThighContactPoints()
   {
      return null;
   }

   @Override
   public List<Pair<String, Vector3d>> getJointNameGroundContactPointMap()
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
   public SideDependentList<Transform3d> getHandContactPointTransforms()
   {
      return null;
   }

   @Override
   public SideDependentList<List<Point2d>> getHandContactPoints()
   {
      return null;
   }
}
