package us.ihmc.manipulation.planning.manifold;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.ReachingManifoldMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.ReachingManifoldCommand;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This test tests whether manifold is well constructed by visualizing.
 */
public class ReachingManifoldVisualizingTest
{
   private enum ManifoldType
   {
      Sphere, Cylinder, Torus;
   }

   private static final ManifoldType type = ManifoldType.Torus;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 0.05;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);

   private final YoDouble currentTrajectoryTime = new YoDouble("CurrentTime", registry);

   private final Random random = new Random(1);

   // define manifold
   private final Point3D manifoldOriginPosition = new Point3D(0.7, -0.2, 1.0);
   private final RotationMatrix manifoldOriginOrientation = new RotationMatrix();

   // define initial hand configuration
   private final Point3D initialHandPosition = new Point3D(0.1, -0.4, 0.8);
   private final double[] handLowerLimits = new double[] {1.0, 0.5, 0.4};
   private final double[] handUpperLimits = new double[] {-0.2, -0.3, -0.2};

   public ReachingManifoldVisualizingTest()
   {
      // scs
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("voidrobot"), parameters);

      // hand frame
      RigidBodyBasics hand = new RigidBody("hand", new RigidBodyTransform(), worldFrame);

      // create manifold message
      List<ReachingManifoldMessage> reachingManifolds = null;
      switch (type)
      {
      case Sphere:
         reachingManifolds = ReachingManifoldTools.createSphereManifoldMessagesForValkyrie(RobotSide.LEFT, hand, manifoldOriginPosition, 0.1);
         break;
      case Cylinder:
         reachingManifolds = ReachingManifoldTools.createCylinderManifoldMessagesForValkyrie(RobotSide.LEFT, hand, manifoldOriginPosition,
                                                                                             manifoldOriginOrientation, 0.2, 0.3);
         break;
      case Torus:
         reachingManifolds = ReachingManifoldTools.createTorusManifoldMessagesForValkyrie(RobotSide.LEFT, hand, manifoldOriginPosition,
                                                                                          manifoldOriginOrientation, 0.3, 0.025);
         break;
      }

      // yographics 
      final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      // graphics for manifold message
      for (int i = 0; i < reachingManifolds.size(); i++)
         scs.addStaticLinkGraphics(ReachingManifoldTools.createManifoldMessageStaticGraphic(reachingManifolds.get(i), 0.01, 10));

      List<ReachingManifoldCommand> manifolds = new ArrayList<>();
      for (int i = 0; i < reachingManifolds.size(); i++)
      {
         ReachingManifoldCommand manifold = new ReachingManifoldCommand();
         manifold.setFromMessage(reachingManifolds.get(i));
         manifolds.add(manifold);
      }

      // graphics for hand frame
      YoDouble yoPXHand = new YoDouble("yoPXHand", registry);
      YoDouble yoPYHand = new YoDouble("yoPYHand", registry);
      YoDouble yoPZHand = new YoDouble("yoPZHand", registry);
      YoDouble yoYawHand = new YoDouble("yoYawHand", registry);
      YoDouble yoPitchHand = new YoDouble("yoPitchHand", registry);
      YoDouble yoRollHand = new YoDouble("yoRollHand", registry);
      YoFramePoint3D yoFramePointHand = new YoFramePoint3D(yoPXHand, yoPYHand, yoPZHand, worldFrame);
      YoFrameYawPitchRoll yoFrameYawPitchRollHand = new YoFrameYawPitchRoll(yoYawHand, yoPitchHand, yoRollHand, worldFrame);

      yoGraphicsListRegistry.registerYoGraphic("handViz", new YoGraphicCoordinateSystem("handViz", yoFramePointHand, yoFrameYawPitchRollHand, 0.2));

      // graphics for closest frame
      YoDouble yoPX = new YoDouble("yoPX", registry);
      YoDouble yoPY = new YoDouble("yoPY", registry);
      YoDouble yoPZ = new YoDouble("yoPZ", registry);
      YoDouble yoYaw = new YoDouble("yoYaw", registry);
      YoDouble yoPitch = new YoDouble("yoPitch", registry);
      YoDouble yoRoll = new YoDouble("yoRoll", registry);
      YoFramePoint3D yoFramePoint = new YoFramePoint3D(yoPX, yoPY, yoPZ, worldFrame);
      YoFrameYawPitchRoll yoFrameYawPitchRoll = new YoFrameYawPitchRoll(yoYaw, yoPitch, yoRoll, worldFrame);

      yoGraphicsListRegistry.registerYoGraphic("pointViz", new YoGraphicCoordinateSystem("closestPointViz", yoFramePoint, yoFrameYawPitchRoll, 0.2));

      // run scs
      scs.addYoRegistry(registry);
      scs.setDT(dt, recordFrequency);
      Graphics3DObject worldFrameGraphics = new Graphics3DObject();
      worldFrameGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(worldFrameGraphics);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         currentTrajectoryTime.set(t);

         // random hand position
         yoPXHand.set(initialHandPosition.getX() + random(handLowerLimits[0], handUpperLimits[0]));
         yoPYHand.set(initialHandPosition.getY() + random(handLowerLimits[1], handUpperLimits[1]));
         yoPZHand.set(initialHandPosition.getZ() + random(handLowerLimits[2], handUpperLimits[2]));

         // random hand orientation
         YawPitchRoll randomYPR = new YawPitchRoll();
         Quaternion randomQuat = new Quaternion();

         double s = random(0, 1);
         double s1 = Math.sqrt(1 - s);
         double s2 = Math.sqrt(s);

         double theta1 = Math.PI * 2 * random(0, 1);
         double theta2 = Math.PI * 2 * random(0, 1);

         randomQuat.set(Math.sin(theta1) * s1, Math.cos(theta1) * s1, Math.sin(theta2) * s2, Math.cos(theta2) * s2);
         randomQuat.norm();

         YawPitchRollConversion.convertQuaternionToYawPitchRoll(randomQuat, randomYPR);

         yoYawHand.set(randomYPR.getYaw());
         yoPitchHand.set(randomYPR.getPitch());
         yoRollHand.set(randomYPR.getRoll());

         // find closest frame
         RigidBodyTransform handTransform = new RigidBodyTransform();
         handTransform.appendTranslation(yoPXHand.getDoubleValue(), yoPYHand.getDoubleValue(), yoPZHand.getDoubleValue());
         handTransform.getRotation().setYawPitchRoll(yoYawHand.getDoubleValue(), yoPitchHand.getDoubleValue(), yoRollHand.getDoubleValue());

         RigidBodyTransform closestTransform = new RigidBodyTransform();
         ReachingManifoldTools.packClosestRigidBodyTransformOnManifold(manifolds, handTransform, closestTransform, 1.0, 0.1);

         yoPX.set(closestTransform.getTranslationX());
         yoPY.set(closestTransform.getTranslationY());
         yoPZ.set(closestTransform.getTranslationZ());

         YawPitchRoll closestYPR = new YawPitchRoll();
         YawPitchRollConversion.convertMatrixToYawPitchRoll(closestTransform.getRotation(), closestYPR);

         yoYaw.set(closestYPR.getYaw());
         yoPitch.set(closestYPR.getPitch());
         yoRoll.set(closestYPR.getRoll());

         // update scs
         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new ReachingManifoldVisualizingTest();
   }

   private double random(double lowerLimit, double upperLimit)
   {
      return random.nextDouble() * (upperLimit - lowerLimit) + lowerLimit;
   }
}