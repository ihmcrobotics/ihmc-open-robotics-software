package us.ihmc.wholeBodyController;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.partNames.ContactPointDefinitionHolder;
import us.ihmc.robotics.partNames.LeggedJointNameMap;
import us.ihmc.robotics.robotSide.RobotSegment;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.simulationconstructionset.util.BidirectionGroundContactModel;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public abstract class RobotContactPointParameters<E extends Enum<E> & RobotSegment<E>> implements ContactPointDefinitionHolder
{
   private final LeggedJointNameMap<E> jointMap;
   private final double footWidth, toeWidth, footLength;

   private final SegmentDependentList<E, RigidBodyTransform> soleToAnkleFrameTransforms;

   private final SegmentDependentList<E, ArrayList<Point2D>> controllerFootGroundContactPoints;;
   private final SegmentDependentList<E, Point2D> controllerToeContactPoints;
   private final SegmentDependentList<E, LineSegment2D> controllerToeContactLines;

   private final List<ImmutablePair<String, Vector3D>> simulationGroundContactPoints = new ArrayList<>();
   private final E[] robotSegments;

   protected final ArrayList<String> additionalContactRigidBodyNames = new ArrayList<>();
   protected final ArrayList<String> additionalContactNames = new ArrayList<>();
   protected final ArrayList<RigidBodyTransform> additionalContactTransforms = new ArrayList<>();

   private boolean useSoftGroundContactParameters;

   public RobotContactPointParameters(LeggedJointNameMap<E> jointMap, double footWidth, double footLength,
                                      SegmentDependentList<E, RigidBodyTransform> soleToAnkleFrameTransforms)
   {
      this(jointMap, footWidth, footWidth, footLength, soleToAnkleFrameTransforms);
   }

   public RobotContactPointParameters(LeggedJointNameMap<E> jointMap, double toeWidth, double footWidth, double footLength,
                                      SegmentDependentList<E, RigidBodyTransform> soleToAnkleFrameTransforms)
   {
      this.jointMap = jointMap;
      this.toeWidth = toeWidth;
      this.footWidth = footWidth;
      this.footLength = footLength;
      this.soleToAnkleFrameTransforms = soleToAnkleFrameTransforms;
      this.robotSegments = jointMap.getRobotSegments();

      Class<E> clazz = robotSegments[0].getClassType();
      controllerFootGroundContactPoints = new SegmentDependentList<>(clazz);
      controllerToeContactPoints = new SegmentDependentList<>(clazz);
      controllerToeContactLines = new SegmentDependentList<>(clazz);
   }

   protected void createFootContactPoints(FootContactPoints<E> footContactPoints)
   {
      Map<String, List<Tuple3DBasics>> simulationContactPoints = footContactPoints.getSimulationContactPoints(footLength, footWidth, toeWidth, jointMap,
                                                                                                              soleToAnkleFrameTransforms);
      for (String parentJointName : simulationContactPoints.keySet())
      {
         List<Tuple3DBasics> points = simulationContactPoints.get(parentJointName);
         for (Tuple3DBasics point : points)
            addSimulationContactPoint(parentJointName, point);
      }

      SegmentDependentList<E, List<Tuple2DBasics>> controllerContactPoints = footContactPoints.getControllerContactPoints(footLength, footWidth, toeWidth);
      for (E segment : robotSegments)
      {
         List<Tuple2DBasics> points = controllerContactPoints.get(segment);
         controllerFootGroundContactPoints.put(segment, new ArrayList<>());
         for (Tuple2DBasics point : points)
            controllerFootGroundContactPoints.get(segment).add(new Point2D(point));
      }

      SegmentDependentList<E, Tuple2DBasics> toeOffContactPoints = footContactPoints.getToeOffContactPoints(footLength, footWidth, toeWidth);
      SegmentDependentList<E, LineSegment2D> toeOffContactLines = footContactPoints.getToeOffContactLines(footLength, footWidth, toeWidth);
      for (E segment : robotSegments)
      {
         if (toeOffContactPoints != null && toeOffContactLines != null)
         {
            controllerToeContactPoints.put(segment, new Point2D(toeOffContactPoints.get(segment)));
            controllerToeContactLines.put(segment, new LineSegment2D(toeOffContactLines.get(segment)));
         }
      }

      useSoftGroundContactParameters = footContactPoints.useSoftContactPointParameters();
   }

   protected void createDefaultFootContactPoints()
   {
      createFootContactPoints(new DefaultFootContactPoints<>(robotSegments));
   }

   protected final void clearSimulationContactPoints()
   {
      simulationGroundContactPoints.clear();
   }

   protected final void clearControllerFootContactPoints()
   {
      for (E segment : robotSegments)
         controllerFootGroundContactPoints.get(segment).clear();
   }

   protected final void addSimulationContactPoint(String parentJointName, Tuple3DBasics contactPointPositionInParentJointFrame)
   {
      simulationGroundContactPoints.add(new ImmutablePair<String, Vector3D>(parentJointName, new Vector3D(contactPointPositionInParentJointFrame)));
   }

   protected final void addControllerFootContactPoint(E segment, Point2D contactPoint)
   {
      controllerFootGroundContactPoints.get(segment).add(contactPoint);
   }

   protected final void setControllerFootContactPoint(E segment, List<Point2D> contactPoints)
   {
      controllerFootGroundContactPoints.get(segment).clear();
      controllerFootGroundContactPoints.get(segment).addAll(contactPoints);
   }

   protected final void setControllerToeContactPoint(E segment, Point2D toeContactPoint)
   {
      controllerToeContactPoints.get(segment).set(toeContactPoint);
   }

   protected final void setControllerToeContactLine(E segment, LineSegment2D toeContactLine)
   {
      controllerToeContactLines.get(segment).set(toeContactLine);
   }

   public final List<ImmutablePair<String, Vector3D>> getJointNameGroundContactPointMap()
   {
      return simulationGroundContactPoints;
   }

   public SegmentDependentList<E, ArrayList<Point2D>> getFootContactPoints()
   {
      return controllerFootGroundContactPoints;
   }

   public SegmentDependentList<E, ArrayList<Point2D>> getControllerFootGroundContactPoints()
   {
      return controllerFootGroundContactPoints;
   }

   public SegmentDependentList<E, LineSegment2D> getControllerToeContactLines()
   {
      return controllerToeContactLines;
   }

   public SegmentDependentList<E, Point2D> getControllerToeContactPoints()
   {
      return controllerToeContactPoints;
   }

   public ArrayList<String> getAdditionalContactRigidBodyNames()
   {
      return additionalContactRigidBodyNames;
   }

   public ArrayList<RigidBodyTransform> getAdditionalContactTransforms()
   {
      return additionalContactTransforms;
   }

   public ArrayList<String> getAdditionalContactNames()
   {
      return additionalContactNames;
   }

   public void setupGroundContactModelParameters(LinearGroundContactModel linearGroundContactModel)
   {
      setupGroundContactModelParameters(linearGroundContactModel, 0.0001);
   }

   public void setupGroundContactModelParameters(LinearGroundContactModel linearGroundContactModel, double simDT)
   {
      // The gains were computed for simDT = 0.0001sec. This assumes that the gains should be inversely proportional to the simulation DT.
      double simDTRef = 0.0001;
      double modelScale = Math.pow(jointMap.getModelScale(), jointMap.getMassScalePower());

      if (useSoftGroundContactParameters)
      {
         double scale = modelScale * Math.pow(simDTRef / simDT, 0.25);
         linearGroundContactModel.setZStiffness(4000.0 * scale);
         linearGroundContactModel.setZDamping(750.0 * scale);
         linearGroundContactModel.setXYStiffness(50000.0 * scale);
         linearGroundContactModel.setXYDamping(1000.0 * scale);
      }
      else
      {
         double scale = modelScale * Math.pow(simDTRef / simDT, 0.6);

         linearGroundContactModel.setZStiffness(2000.0 * scale);
         linearGroundContactModel.setZDamping(1500.0 * scale);
         linearGroundContactModel.setXYStiffness(50000.0 * scale);
         linearGroundContactModel.setXYDamping(2000.0 * scale);
      }
   }

   public void setupGroundContactModelParameters(BidirectionGroundContactModel bidirectionGroundContactModel)
   {
      double modelScale = Math.pow(jointMap.getModelScale(), jointMap.getMassScalePower());

      if (useSoftGroundContactParameters)
      {
         bidirectionGroundContactModel.setZStiffness(modelScale * 4000.0);
         bidirectionGroundContactModel.setZDamping(modelScale * 750.0);
         bidirectionGroundContactModel.setXYStiffness(modelScale * 50000.0);
         bidirectionGroundContactModel.setXYDamping(modelScale * 1000.0);
      }
      else
      {
         bidirectionGroundContactModel.setZStiffness(modelScale * 2000.0);
         bidirectionGroundContactModel.setZDamping(modelScale * 1500.0);
         bidirectionGroundContactModel.setXYStiffness(modelScale * 50000.0);
         bidirectionGroundContactModel.setXYDamping(modelScale * 2000.0);
      }
   }
}
