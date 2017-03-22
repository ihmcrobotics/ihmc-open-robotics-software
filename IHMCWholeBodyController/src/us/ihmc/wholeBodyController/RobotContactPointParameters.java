package us.ihmc.wholeBodyController;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.partNames.ContactPointDefinitionHolder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.BidirectionGroundContactModel;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public abstract class RobotContactPointParameters implements ContactPointDefinitionHolder
{
   protected final ContactableBodiesFactory contactableBodiesFactory = new ContactableBodiesFactory();

   private final DRCRobotJointMap jointMap;
   private final double footWidth, toeWidth, footLength;
   private final SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms;

   protected final SideDependentList<ArrayList<Point2D>> controllerFootGroundContactPoints = new SideDependentList<>();
   protected final SideDependentList<Point2D> controllerToeContactPoints = new SideDependentList<>();

   private final List<ImmutablePair<String, Vector3D>> simulationGroundContactPoints = new ArrayList<ImmutablePair<String, Vector3D>>();

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

   protected void createFootContactPoints(FootContactPoints footContactPoints)
   {
      Map<String, List<Tuple3DBasics>> simulationContactPoints = footContactPoints.getSimulationContactPoints(footLength, footWidth, toeWidth, jointMap, soleToAnkleFrameTransforms);
      for (String parentJointName : simulationContactPoints.keySet())
      {
         List<Tuple3DBasics> points = simulationContactPoints.get(parentJointName);
         for (Tuple3DBasics point : points)
            addSimulationContactPoint(parentJointName, point);
      }


      SideDependentList<List<Tuple2DBasics>> controllerContactPoints = footContactPoints.getControllerContactPoints(footLength, footWidth, toeWidth);
      for (RobotSide robotSide : RobotSide.values)
      {
         List<Tuple2DBasics> points = controllerContactPoints.get(robotSide);
         controllerFootGroundContactPoints.put(robotSide, new ArrayList<Point2D>());
         for (Tuple2DBasics point : points)
            controllerFootGroundContactPoints.get(robotSide).add(new Point2D(point));
      }

      SideDependentList<Tuple2DBasics> toeOffContactPoints = footContactPoints.getToeOffContactPoints(footLength, footWidth, toeWidth);
      for (RobotSide robotSide : RobotSide.values)
      {
         controllerToeContactPoints.put(robotSide, new Point2D(toeOffContactPoints.get(robotSide)));
      }

      contactableBodiesFactory.addFootContactParameters(controllerFootGroundContactPoints, controllerToeContactPoints);

      useSoftGroundContactParameters = footContactPoints.useSoftContactPointParameters();
   }

   protected void createDefaultFootContactPoints()
   {
      createFootContactPoints(new DefaultFootContactPoints());
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

   protected final void addSimulationContactPoint(String parentJointName, Tuple3DBasics contactPointPositionInParentJointFrame)
   {
      simulationGroundContactPoints.add(new ImmutablePair<String, Vector3D>(parentJointName, new Vector3D(contactPointPositionInParentJointFrame)));
   }

   protected final void addControllerFootContactPoint(RobotSide robotSide, Point2D contactPoint)
   {
      controllerFootGroundContactPoints.get(robotSide).add(contactPoint);
      // Update the factory with the new set of contact points.
      contactableBodiesFactory.addFootContactParameters(controllerFootGroundContactPoints, controllerToeContactPoints);
   }

   protected final void setControllerFootContactPoint(RobotSide robotSide, List<Point2D> contactPoints)
   {
      controllerFootGroundContactPoints.get(robotSide).clear();
      controllerFootGroundContactPoints.get(robotSide).addAll(contactPoints);
      // Update the factory with the new set of contact points.
      contactableBodiesFactory.addFootContactParameters(controllerFootGroundContactPoints, controllerToeContactPoints);
   }

   protected final void setControllerToeContactPoint(RobotSide robotSide, Point2D toeContactPoint)
   {
      controllerToeContactPoints.get(robotSide).set(toeContactPoint);
      // Update the factory with the new set of contact points.
      contactableBodiesFactory.addFootContactParameters(controllerFootGroundContactPoints, controllerToeContactPoints);
   }

   public final List<ImmutablePair<String, Vector3D>> getJointNameGroundContactPointMap()
   {
      return simulationGroundContactPoints;
   }

   public SideDependentList<ArrayList<Point2D>> getFootContactPoints()
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
