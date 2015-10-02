package us.ihmc.darpaRoboticsChallenge.visualization;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.RectangularContactableBody;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepUtils;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.SimplePathParameters;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.TurningThenStraightFootstepGenerator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPolygon;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class FootstepGeneratorVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry("FootstepGeneratorVisualizer");

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final boolean DEBUG = false;

   private final ArrayList<YoFramePose> contactPoses = new ArrayList<YoFramePose>();
   private final ArrayList<YoFrameConvexPolygon2d> contactPolygonsWorld = new ArrayList<YoFrameConvexPolygon2d>();

   private final LinkedHashMap<String, YoFramePose> contactPosesHashMap = new LinkedHashMap<String, YoFramePose>();
   private final LinkedHashMap<String, YoFrameConvexPolygon2d> contactPolygonsHashMap = new LinkedHashMap<String, YoFrameConvexPolygon2d>();

   public FootstepGeneratorVisualizer(int maxNumberOfContacts, int maxPointsPerContact, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList("FootstepGeneratorVisualizer");

      AppearanceDefinition[] appearances = new AppearanceDefinition[] { YoAppearance.Red(), YoAppearance.Green(), YoAppearance.Blue(), YoAppearance.Purple() };

      for (int i = 0; i < maxNumberOfContacts; i++)
      {
         YoFramePose contactPose = new YoFramePose("contactPose" + i, "", worldFrame, registry);
         contactPoses.add(contactPose);

         YoFrameConvexPolygon2d contactPolygon = new YoFrameConvexPolygon2d("contactPolygon" + i, "", worldFrame, maxPointsPerContact, registry);
         contactPolygonsWorld.add(contactPolygon);

         YoGraphicPolygon dynamicGraphicPolygon = new YoGraphicPolygon("contactPolygon" + i, contactPolygon, contactPose, 1.0, appearances[i
               % appearances.length]);
         yoGraphicsList.add(dynamicGraphicPolygon);
      }

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      parentRegistry.addChild(registry);
   }

   public void addFootstepsAndTickAndUpdate(SimulationConstructionSet scs, List<Footstep> footsteps, List<ContactablePlaneBody> contactablePlaneBodies)
   {
      for (int i = 0; i < footsteps.size(); i++)
      {
         Footstep footstep = footsteps.get(i);
         ContactablePlaneBody contactablePlaneBody = contactablePlaneBodies.get(i);
         addFootstep(footstep, contactablePlaneBody);
         scs.tickAndUpdate();
      }

   }

   public void addFootstep(Footstep footstep, ContactablePlaneBody contactablePlaneBody)
   {
      printIfDebug("Adding footstep " + footstep);

      FramePose footstepPose = new FramePose();
      footstep.getPose(footstepPose);

      String name = footstep.getBody().getName();
      YoFramePose contactPose = contactPosesHashMap.get(name);
      YoFrameConvexPolygon2d contactPolygon = contactPolygonsHashMap.get(name);

      if (contactPose == null)
      {
         printIfDebug("Associating new rigidBody " + name);
         contactPose = contactPoses.remove(0);
         contactPolygon = contactPolygonsWorld.remove(0);

         contactPosesHashMap.put(name, contactPose);
         contactPolygonsHashMap.put(name, contactPolygon);

         // Keep the contact points in sole frame:
         List<FramePoint> expectedContactPoints = FootstepUtils.calculateExpectedContactPoints(footstep, contactablePlaneBody);
         ArrayList<Point2d> contactPointsInSoleFrame = new ArrayList<Point2d>();

         for (FramePoint contactPoint : expectedContactPoints)
         {
            contactPointsInSoleFrame.add(new Point2d(contactPoint.getX(), contactPoint.getY()));
         }

         ConvexPolygon2d convexPolygon2d = new ConvexPolygon2d(contactPointsInSoleFrame);
         contactPolygon.setConvexPolygon2d(convexPolygon2d);
      }

      else
      {
         printIfDebug("Found association for rigidBody " + name);
      }

      FramePose footstepPoseInWorld = new FramePose(footstepPose);
      footstepPoseInWorld.changeFrame(worldFrame);
      FramePoint position = new FramePoint();
      footstepPoseInWorld.getPositionIncludingFrame(position);
      position.setZ(position.getZ() + 0.0005);
      footstepPoseInWorld.setPosition(position);

      contactPose.set(footstepPoseInWorld);
   }

   private void printIfDebug(String string)
   {
      if (DEBUG)
         System.out.println(string);
   }

   public static void main(String[] args)
   {
      Robot nullRobot = new Robot("FootstepVisualizerRobot");

      ImmutablePair<List<Footstep>, List<ContactablePlaneBody>> footstepsAndContactablePlaneBodies = generateDefaultFootstepList();

      visualizeFootsteps(nullRobot, footstepsAndContactablePlaneBodies.getLeft(), footstepsAndContactablePlaneBodies.getRight());
   }

   public static ImmutablePair<List<Footstep>, List<ContactablePlaneBody>> generateDefaultFootstepList()
   {
      SideDependentList<ContactablePlaneBody> contactableFeet = new SideDependentList<ContactablePlaneBody>();
      SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<ReferenceFrame>();
      SideDependentList<RigidBody> feet = new SideDependentList<RigidBody>();
      SideDependentList<SixDoFJoint> sixDoFJoints = new SideDependentList<SixDoFJoint>();

      RigidBody rootBody = new RigidBody("Root", ReferenceFrame.getWorldFrame());
      Random random = new Random(1776L);

      for (RobotSide robotSide : RobotSide.values)
      {
         PoseReferenceFrame soleFrame = new PoseReferenceFrame(robotSide.getCamelCaseNameForStartOfExpression() + "Frame", worldFrame);
         soleFrames.set(robotSide, soleFrame);

         SixDoFJoint sixDoFJoint = new SixDoFJoint("SixDofJoint" + robotSide, rootBody, rootBody.getBodyFixedFrame());
         sixDoFJoints.set(robotSide, sixDoFJoint);

         RigidBody rigidBody = ScrewTestTools.addRandomRigidBody(robotSide.getCamelCaseNameForStartOfExpression() + "Foot", random, sixDoFJoint);
         feet.put(robotSide, rigidBody);
         double forward = 0.12;
         double back = -0.07;
         double left = 0.06;
         double right = -0.06;
         RectangularContactableBody contactableBody = new RectangularContactableBody(rigidBody, soleFrame, forward, back, left, right);

         contactableFeet.set(robotSide, contactableBody);
      }

      //      FramePose2d startPose = new FramePose2d(worldFrame, new Point2d(0.0, 0.0), Math.PI);//should be done in bipedFeet instead
      FramePoint2d endPoint = new FramePoint2d(worldFrame, new Point2d(0.0, -3.0));

      SimplePathParameters pathType = new SimplePathParameters(0.4, 0.2, 0.0, Math.PI * 0.8, Math.PI * 0.15, 0.35);

      TurningThenStraightFootstepGenerator generator = new TurningThenStraightFootstepGenerator(feet, soleFrames, endPoint, pathType, RobotSide.LEFT);
      List<Footstep> footsteps = generator.generateDesiredFootstepList();

      ImmutablePair<List<Footstep>, List<ContactablePlaneBody>> footstepsAndContactablePlaneBodies = new ImmutablePair<List<Footstep>, List<ContactablePlaneBody>>(
            new ArrayList<Footstep>(), new ArrayList<ContactablePlaneBody>());

      for (Footstep footstep : footsteps)
      {
         footstepsAndContactablePlaneBodies.getLeft().add(footstep);
         footstepsAndContactablePlaneBodies.getRight().add(contactableFeet.get(footstep.getRobotSide()));
      }

      return footstepsAndContactablePlaneBodies;
   }

   public static FramePose setupStanceFoot(SideDependentList<PoseReferenceFrame> soleFrames, SideDependentList<SixDoFJoint> sixDoFJoints)
   {
      FramePose startStanceFootPose = new FramePose(worldFrame, new Point3d(0.0, 0.2, 0.0), new Quat4d());
      soleFrames.get(RobotSide.LEFT).setPoseAndUpdate(startStanceFootPose);
      soleFrames.get(RobotSide.LEFT).update();
      Point3d stanceAnklePosition = new Point3d();
      startStanceFootPose.getPosition(stanceAnklePosition);
      sixDoFJoints.get(RobotSide.LEFT).setPosition(stanceAnklePosition);

      return startStanceFootPose;
   }

   public static FramePose setupSwingFoot(SideDependentList<PoseReferenceFrame> soleFrames, SideDependentList<SixDoFJoint> sixDoFJoints)
   {
      FramePose startSwingFootPose = new FramePose(worldFrame, new Point3d(0.0, -0.2, 0.0), new Quat4d());
      soleFrames.get(RobotSide.RIGHT).setPoseAndUpdate(startSwingFootPose);
      soleFrames.get(RobotSide.RIGHT).update();
      Point3d swingAnklePosition = new Point3d();
      startSwingFootPose.getPosition(swingAnklePosition);
      sixDoFJoints.get(RobotSide.RIGHT).setPosition(swingAnklePosition);

      return startSwingFootPose;
   }

   public static void visualizeFootsteps(Robot nullRobot, List<Footstep> footsteps, List<ContactablePlaneBody> contactablePlaneBodies)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(nullRobot);
      scs.setDT(0.25, 1);
      YoVariableRegistry rootRegistry = scs.getRootRegistry();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      int maxNumberOfContacts = 2;
      int maxPointsPerContact = 4;
      FootstepGeneratorVisualizer footstepGeneratorVisualizer = new FootstepGeneratorVisualizer(maxNumberOfContacts, maxPointsPerContact, rootRegistry,
            yoGraphicsListRegistry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      footstepGeneratorVisualizer.addFootstepsAndTickAndUpdate(scs, footsteps, contactablePlaneBodies);

      scs.startOnAThread();

      deleteFirstDataPointAndCropData(scs);
   }

   private static void deleteFirstDataPointAndCropData(SimulationConstructionSet scs)
   {
      scs.gotoInPointNow();
      scs.tick(1);
      scs.setInPoint();
      scs.cropBuffer();
   }

}
