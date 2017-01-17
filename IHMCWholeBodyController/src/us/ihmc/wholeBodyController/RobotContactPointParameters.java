package us.ihmc.wholeBodyController;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.BidirectionGroundContactModel;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public abstract class RobotContactPointParameters
{
   protected final ContactableBodiesFactory contactableBodiesFactory = new ContactableBodiesFactory();

   private final DRCRobotJointMap jointMap;
   private final double footWidth, toeWidth, footLength;
   private final SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms;

   protected final SideDependentList<ArrayList<Point2d>> controllerFootGroundContactPoints = new SideDependentList<>();
   protected final SideDependentList<Point2d> controllerToeContactPoints = new SideDependentList<>();

   private final List<ImmutablePair<String, Vector3d>> simulationGroundContactPoints = new ArrayList<ImmutablePair<String, Vector3d>>();

   private boolean useSoftGroundContactParameters;

   public RobotContactPointParameters(DRCRobotJointMap jointMap, double footWidth, double footLength, SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms)
   {
      this(jointMap, footWidth, footWidth, footLength, soleToAnkleFrameTransforms);
   }

   public RobotContactPointParameters(DRCRobotJointMap jointMap, double toeWidth, double footWidth, double footLength, SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms)
   {
      this.jointMap = jointMap;
      this.toeWidth = toeWidth;
      this.footWidth = footWidth;
      this.footLength = footLength;
      this.soleToAnkleFrameTransforms = soleToAnkleFrameTransforms;
   }

   protected void createDefaultControllerFootContactPoints()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         controllerFootGroundContactPoints.put(robotSide, new ArrayList<Point2d>());
         controllerFootGroundContactPoints.get(robotSide).add(new Point2d(-footLength / 2.0, -footWidth / 2.0));
         controllerFootGroundContactPoints.get(robotSide).add(new Point2d(-footLength / 2.0, footWidth / 2.0));
         controllerFootGroundContactPoints.get(robotSide).add(new Point2d(footLength / 2.0, -toeWidth / 2.0));
         controllerFootGroundContactPoints.get(robotSide).add(new Point2d(footLength / 2.0, toeWidth / 2.0));
         controllerToeContactPoints.put(robotSide, new Point2d(footLength / 2.0, 0.0));
      }

      contactableBodiesFactory.addFootContactParameters(controllerFootGroundContactPoints, controllerToeContactPoints);
   }

   protected void createSimulationContactPoints(SimulationFootContactPoints simulationContactPoints)
   {
      Map<String, List<Tuple3d>> contactPoints = simulationContactPoints.getContactPoints(footLength, footWidth, toeWidth, jointMap, soleToAnkleFrameTransforms);
      for (String parentJointName : contactPoints.keySet())
      {
         List<Tuple3d> points = contactPoints.get(parentJointName);
         for (Tuple3d point : points)
            addSimulationContactPoint(parentJointName, point);
      }

      useSoftGroundContactParameters = simulationContactPoints.useSoftContactPointParameters();
   }

   protected void createDefaultSimulationFootContactPoints()
   {
      createSimulationContactPoints(new DefaultSimulationContactPoints());
   }

   protected final void clearSimulationContactPoints()
   {
      simulationGroundContactPoints.clear();
   }

   protected final void clearControllerFootContactPoints()
   {
      for (RobotSide robotSide : RobotSide.values)
         controllerFootGroundContactPoints.get(robotSide).clear();
   }

   protected final void addSimulationContactPoint(String parentJointName, RigidBodyTransform transformToParentJointFrame, double contactPointX, double contactPointY, double contactPointZ)
   {
      Point3d contactPoint = new Point3d(contactPointX, contactPointY, contactPointZ);
      transformToParentJointFrame.transform(contactPoint);
      addSimulationContactPoint(parentJointName, contactPoint);
   }

   protected final void addSimulationContactPoint(String parentJointName, RigidBodyTransform transformToParentJointFrame, Point2d contactPointPosition)
   {
      Point3d contactPoint = new Point3d(contactPointPosition.getX(), contactPointPosition.getY(), 0.0);
      transformToParentJointFrame.transform(contactPoint);
      addSimulationContactPoint(parentJointName, contactPoint);
   }

   protected final void addSimulationContactPoint(String parentJointName, Tuple3d contactPointPositionInParentJointFrame)
   {
      simulationGroundContactPoints.add(new ImmutablePair<String, Vector3d>(parentJointName, new Vector3d(contactPointPositionInParentJointFrame)));
   }

   protected final void addControllerFootContactPoint(RobotSide robotSide, Point2d contactPoint)
   {
      controllerFootGroundContactPoints.get(robotSide).add(contactPoint);
      // Update the factory with the new set of contact points.
      contactableBodiesFactory.addFootContactParameters(controllerFootGroundContactPoints, controllerToeContactPoints);
   }

   protected final void setControllerFootContactPoint(RobotSide robotSide, List<Point2d> contactPoints)
   {
      controllerFootGroundContactPoints.get(robotSide).clear();
      controllerFootGroundContactPoints.get(robotSide).addAll(contactPoints);
      // Update the factory with the new set of contact points.
      contactableBodiesFactory.addFootContactParameters(controllerFootGroundContactPoints, controllerToeContactPoints);
   }

   protected final void setControllerToeContactPoint(RobotSide robotSide, Point2d toeContactPoint)
   {
      controllerToeContactPoints.get(robotSide).set(toeContactPoint);
      // Update the factory with the new set of contact points.
      contactableBodiesFactory.addFootContactParameters(controllerFootGroundContactPoints, controllerToeContactPoints);
   }

   public SideDependentList<RigidBodyTransform> getHandContactPointTransforms()
   {
      return null;
   }

   public SideDependentList<List<Point2d>> getHandContactPoints()
   {
      return null;
   }

   public final List<ImmutablePair<String, Vector3d>> getJointNameGroundContactPointMap()
   {
      return simulationGroundContactPoints;
   }

   public SideDependentList<ArrayList<Point2d>> getFootContactPoints()
   {
      return controllerFootGroundContactPoints;
   }

   public final ContactableBodiesFactory getContactableBodiesFactory()
   {
      return contactableBodiesFactory;
   }

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

   public void setupGroundContactModelParameters(BidirectionGroundContactModel bidirectionGroundContactModel)
   {
      if (useSoftGroundContactParameters)
      {
         bidirectionGroundContactModel.setZStiffness(4000.0);
         bidirectionGroundContactModel.setZDamping(750.0);
         bidirectionGroundContactModel.setXYStiffness(50000.0);
         bidirectionGroundContactModel.setXYDamping(1000.0);
      }
      else
      {
         bidirectionGroundContactModel.setZStiffness(2000.0);
         bidirectionGroundContactModel.setZDamping(1500.0);
         bidirectionGroundContactModel.setXYStiffness(50000.0);
         bidirectionGroundContactModel.setXYDamping(2000.0);
      }
   }
}
