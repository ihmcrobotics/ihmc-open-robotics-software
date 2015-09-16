package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.RotatableConvexPolygonTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class MultiContactTestEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject;
   private final RobotSide[] footContactSides;
   private final RobotSide[] handContactSides;

   public MultiContactTestEnvironment(DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup, DRCRobotModel robotModel,
                                      RobotSide[] footContactSides, RobotSide[] handContactSides, SideDependentList<RigidBodyTransform> invisibleContactablePlaneHandContactPointTransforms)
   {
      DRCRobotJointMap jointMap = robotModel.getJointMap();
      SDFHumanoidRobot robotForEnvironmentSetup = robotModel.createSdfRobot(DRCConfigParameters.USE_COLLISIONS_MESHS_FOR_VISUALIZATION);
      robotInitialSetup.initializeRobot(robotForEnvironmentSetup, jointMap);
      robotForEnvironmentSetup.update();
      FullHumanoidRobotModel fullRobotModelForEnvironmentSetup = robotModel.createFullRobotModel();

      CommonHumanoidReferenceFrames referenceFramesForEnvironmentSetup = new HumanoidReferenceFrames(fullRobotModelForEnvironmentSetup);
      SDFPerfectSimulatedSensorReader sensorReaderAndOutputWriter = new SDFPerfectSimulatedSensorReader(robotForEnvironmentSetup,
                                                                                fullRobotModelForEnvironmentSetup, referenceFramesForEnvironmentSetup);
      sensorReaderAndOutputWriter.read();
      
      this.footContactSides = footContactSides;
      this.handContactSides = handContactSides;

      combinedTerrainObject = createCombinedTerrainObject(referenceFramesForEnvironmentSetup, fullRobotModelForEnvironmentSetup, invisibleContactablePlaneHandContactPointTransforms);
   }

   private CombinedTerrainObject3D createCombinedTerrainObject(CommonHumanoidReferenceFrames referenceFramesForEnvironmentSetup, FullHumanoidRobotModel fullRobotModel, SideDependentList<RigidBodyTransform> invisibleContactablePlaneHandContactPointTransforms)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      for (RobotSide robotSide : footContactSides)
      {
         ReferenceFrame soleFrame = referenceFramesForEnvironmentSetup.getSoleFrame(robotSide);
         RigidBodyTransform transformToWorld = soleFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
         combinedTerrainObject.addTerrainObject(createConvexPolygonTerrainObject(transformToWorld));
      }

      for (RobotSide robotSide : handContactSides)
      {
         ReferenceFrame handFrame = fullRobotModel.getHand(robotSide).getParentJoint().getFrameAfterJoint();
         RigidBodyTransform handToWorld = handFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
         RigidBodyTransform handContactPlaneToWorld = new RigidBodyTransform();
         handContactPlaneToWorld.multiply(handToWorld, invisibleContactablePlaneHandContactPointTransforms.get(robotSide));
         combinedTerrainObject.addTerrainObject(createConvexPolygonTerrainObject(handContactPlaneToWorld));
      }

      return combinedTerrainObject;
   }

   private TerrainObject3D createConvexPolygonTerrainObject(RigidBodyTransform transformToWorld)
   {
      Matrix3d rotationToWorld = new Matrix3d();
      transformToWorld.get(rotationToWorld);

      Vector3d normal = new Vector3d();
      rotationToWorld.getColumn(2, normal);

      Vector3d centroid = new Vector3d();
      transformToWorld.get(centroid);

      int nPoints = 5;
      double radius = 0.23;
      ConvexPolygon2d convexPolygon = createContactPolygon(centroid, nPoints, radius);

      TerrainObject3D contact = new RotatableConvexPolygonTerrainObject(normal, convexPolygon, centroid.getZ(), YoAppearance.DarkGray());

      return contact;
   }

   private ConvexPolygon2d createContactPolygon(Vector3d centroid, int nPoints, double radius)
   {
      List<Point2d> pointList = new ArrayList<Point2d>();
      double angleIncrement = 2.0 * Math.PI / nPoints;
      for (int i = 0; i < nPoints; i++)
      {
         double angle = i * angleIncrement;
         double x = radius * Math.cos(angle) + centroid.getX();
         double y = radius * Math.sin(angle) + centroid.getY();
         pointList.add(new Point2d(x, y));
      }

      ConvexPolygon2d convexPolygon = new ConvexPolygon2d(pointList);

      return convexPolygon;
   }
   
   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   @Override
   public List<Robot> getEnvironmentRobots()
   {
      return new ArrayList<Robot>();
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
      // empty
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      // empty
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
      // empty
   }

}
