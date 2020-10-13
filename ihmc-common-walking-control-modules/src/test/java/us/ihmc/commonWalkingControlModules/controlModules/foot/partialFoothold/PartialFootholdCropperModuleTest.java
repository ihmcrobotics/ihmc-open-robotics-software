package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class PartialFootholdCropperModuleTest
{
   protected RobotSide side;
   protected YoRegistry registry;
   protected double dt;
   protected Twist soleTwist;
   protected Pose3D solePose;
   protected MovingReferenceFrame soleFrame;

   @BeforeEach
   public void setup()
   {
      registry = new YoRegistry(FeetManager.class.getSimpleName());
      dt = 0.001;
      side = RobotSide.LEFT;
      soleTwist = new Twist();
      solePose = new Pose3D();
      soleFrame = new TestSoleFrame(soleTwist, solePose);
      soleTwist.setToZero(soleFrame, ReferenceFrame.getWorldFrame(), soleFrame);
   }

   @AfterEach
   public void tearDown()
   {
      side = null;
      registry = null;
      soleTwist = null;
      solePose = null;
      soleFrame = null;
   }

   @Test
   public void testRotationAndCropping()
   {
      Random random = new Random(429L);

      double footLength = 0.22;
      double footWidth = 0.11;
      FootholdRotationParameters parameters = new FootholdRotationParameters(registry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      List<? extends FramePoint2DReadOnly> defaultFootPolygons = createFootPolygonPoints(soleFrame, footLength, footWidth, footWidth);
      FrameConvexPolygon2DBasics croppedFootPolygon = createFootPolygon(soleFrame, footLength, footWidth, footWidth);
      PartialFootholdCropperModule partialFootholdModule = new PartialFootholdCropperModule(side,
                                                                                            soleFrame,
                                                                                            defaultFootPolygons,
                                                                                            parameters,
                                                                                            dt,
                                                                                            registry,
                                                                                            null);

      YoInteger shrinkMaxLimit = ((YoInteger) registry.findVariable("Cropping_ShrinkMaxLimit"));
      shrinkMaxLimit.set(6);
      YoBoolean applyPartialFootholds = ((YoBoolean) registry.findVariable("applyPartialFootholds"));
      applyPartialFootholds.set(true);


      int historyRequirement = 20;      // create random cop location (zero linear velocity) and a rotational velocity

      partialFootholdModule.initialize(createFootPolygon(soleFrame, footLength, footWidth, footWidth));

      List<FramePoint2D> measuredCoPs = new ArrayList<>();

      // do a little bit of initial points in the middle of the support area
      double diagonalWidth = 0.02;
      for (int i = 0; i < 10; i++)
      {
         double scale = InterpolationTools.linearInterpolate(-diagonalWidth, diagonalWidth, ((double) i) / ((double) historyRequirement));
         FramePoint2D measuredCoP = new FramePoint2D(soleFrame, scale, scale);
         measuredCoPs.add(measuredCoP);

         // assume perfect tracking when the foot is at normal
         partialFootholdModule.compute(measuredCoP, measuredCoP);
      }

      // now add some rotation
      double omegaNorm = random.nextDouble() + 10.0;

      // first do the back half of the foot, rotating up onto the heel
      FramePoint3D pointOfRotation = new FramePoint3D(soleFrame, -0.05, 0.0, 0.0);
      FramePoint2D perfectDesiredCoP = new FramePoint2D(soleFrame, -0.075, 0.0);
      FrameVector3D rotationAngularVelocity = new FrameVector3D(soleFrame, 0.0, -omegaNorm, 0.0);
      Vector2D direction = new Vector2D(rotationAngularVelocity);
      direction.normalize();

      double footPitch = 0.0;
      // update the sole twist based on the cop location and the angular velocity
      soleTwist.setIncludingFrame(rotationAngularVelocity, new FrameVector3D(soleFrame), pointOfRotation);
      soleFrame.update();

      double lineWidth = footWidth / 2.0;
      double desiredLineWidth = footWidth / 4.0;
      for (int increment = 0; increment < historyRequirement; increment++)
      {
         footPitch += -omegaNorm * dt;
         solePose.getOrientation().setYawPitchRoll(0.0, footPitch, 0.0);
         soleFrame.update();

         FramePoint2D measuredCoP = new FramePoint2D(soleFrame, direction);
         double scale = InterpolationTools.linearInterpolate(-lineWidth, lineWidth, ((double) increment) / ((double) historyRequirement));
         measuredCoP.scale(scale);
         measuredCoP.add(pointOfRotation.getX(), pointOfRotation.getY());

         measuredCoPs.add(measuredCoP);

         FramePoint2D desiredCoP = new FramePoint2D(soleFrame, direction);
         double desiredScale = InterpolationTools.linearInterpolate(-desiredLineWidth, desiredLineWidth, ((double) increment) / ((double) historyRequirement));
         desiredCoP.scale(desiredScale);
         desiredCoP.add(perfectDesiredCoP.getX(), perfectDesiredCoP.getY());

         // try to estimate the line of rotation which should go through the cop and have the direction of omega
         partialFootholdModule.compute(measuredCoP, desiredCoP);
      }

      ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

      assertTrue(partialFootholdModule.isRotating());

      FrameLine2DReadOnly lineEstimate = partialFootholdModule.getLineOfRotation();
      FrameLine2D expectedLine = new FrameLine2D(soleFrame);

      expectedLine.getDirection().set(direction);
      expectedLine.getPoint().set(pointOfRotation);
      convexPolygonTools.cutPolygonWithLine(expectedLine, croppedFootPolygon, RobotSide.RIGHT);

      EuclidGeometryTestTools.assertLine2DGeometricallyEquals(expectedLine, lineEstimate, 1.0e-5);
      assertTrue(partialFootholdModule.shouldApplyShrunkenFoothold());
      EuclidGeometryTestTools.assertConvexPolygon2DEquals(croppedFootPolygon, partialFootholdModule.getShrunkenFootPolygon(), 1.0e-5);

      for (FramePoint2DReadOnly measuredCoP : measuredCoPs)
         assertTrue(partialFootholdModule.getShrunkenFootPolygon().signedDistance(measuredCoP) < 1e-3);
   }

   @Disabled
   @Test
   public void testRotationAndCroppingBothDirections()
   {
      Random random = new Random(429L);

      double footLength = 0.22;
      double footWidth = 0.11;
      FootholdRotationParameters parameters = new FootholdRotationParameters(registry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      List<? extends FramePoint2DReadOnly> defaultFootPolygons = createFootPolygonPoints(soleFrame, footLength, footWidth, footWidth);
      FrameConvexPolygon2DBasics croppedFootPolygon = createFootPolygon(soleFrame, footLength, footWidth, footWidth);
      PartialFootholdCropperModule partialFootholdModule = new PartialFootholdCropperModule(side,
                                                                                            soleFrame,
                                                                                            defaultFootPolygons,
                                                                                            parameters,
                                                                                            dt,
                                                                                            registry,
                                                                                            null);

      YoInteger shrinkMaxLimit = ((YoInteger) registry.findVariable("Cropping_ShrinkMaxLimit"));
      shrinkMaxLimit.set(6);
      YoBoolean applyPartialFootholds = ((YoBoolean) registry.findVariable("leftApplyPartialFootholds"));
      applyPartialFootholds.set(true);

      int historyRequirement = 20;      // create random cop location (zero linear velocity) and a rotational velocity

      partialFootholdModule.initialize(createFootPolygon(soleFrame, footLength, footWidth, footWidth));

      List<FramePoint2D> measuredCoPs = new ArrayList<>();

      // do a little bit of initial points in the middle of the support area
      double diagonalWidth = 0.02;
      for (int i = 0; i < 10; i++)
      {
         double scale = InterpolationTools.linearInterpolate(-diagonalWidth, diagonalWidth, ((double) i) / ((double) historyRequirement));
         FramePoint2D measuredCoP = new FramePoint2D(soleFrame, scale, scale);
         measuredCoPs.add(measuredCoP);

         // assume perfect tracking when the foot is at normal
         partialFootholdModule.compute(measuredCoP, measuredCoP);
      }

      // now add some rotation
      double omegaNorm = random.nextDouble() + 10.0;

      // first do the back half of the foot, rotating up onto the heel
      FramePoint3D pointOfRotation = new FramePoint3D(soleFrame, -0.05, 0.0, 0.0);
      FramePoint2D perfectDesiredCoP = new FramePoint2D(soleFrame, -0.075, 0.0);
      FrameVector3D rotationAngularVelocity = new FrameVector3D(soleFrame, 0.0, -omegaNorm, 0.0);
      Vector2D direction = new Vector2D(rotationAngularVelocity);
      direction.normalize();

      double footPitch = 0.0;
      // update the sole twist based on the cop location and the angular velocity
      soleTwist.setIncludingFrame(rotationAngularVelocity, new FrameVector3D(soleFrame), pointOfRotation);
      soleFrame.update();

      double lineWidth = footWidth / 2.0;
      double desiredLineWidth = footWidth / 4.0;
      for (int increment = 0; increment < historyRequirement; increment++)
      {
         footPitch += -omegaNorm * dt;
         solePose.getOrientation().setYawPitchRoll(0.0, footPitch, 0.0);
         soleFrame.update();

         FramePoint2D measuredCoP = new FramePoint2D(soleFrame, direction);
         double scale = InterpolationTools.linearInterpolate(-lineWidth, lineWidth, ((double) increment) / ((double) historyRequirement));
         measuredCoP.scale(scale);
         measuredCoP.add(pointOfRotation.getX(), pointOfRotation.getY());

         measuredCoPs.add(measuredCoP);

         FramePoint2D desiredCoP = new FramePoint2D(soleFrame, direction);
         double desiredScale = InterpolationTools.linearInterpolate(-desiredLineWidth, desiredLineWidth, ((double) increment) / ((double) historyRequirement));
         desiredCoP.scale(desiredScale);
         desiredCoP.add(perfectDesiredCoP.getX(), perfectDesiredCoP.getY());

         // try to estimate the line of rotation which should go through the cop and have the direction of omega
         partialFootholdModule.compute(measuredCoP, desiredCoP);
      }

      ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

      assertTrue(partialFootholdModule.isRotating());

      FrameLine2DReadOnly lineEstimate = partialFootholdModule.getLineOfRotation();
      FrameLine2D expectedLine = new FrameLine2D(soleFrame);

      expectedLine.getDirection().set(direction);
      expectedLine.getPoint().set(pointOfRotation);
      convexPolygonTools.cutPolygonWithLine(expectedLine, croppedFootPolygon, RobotSide.RIGHT);

      EuclidGeometryTestTools.assertLine2DGeometricallyEquals(expectedLine, lineEstimate, 1.0e-5);
      assertTrue(partialFootholdModule.shouldApplyShrunkenFoothold());
      EuclidGeometryTestTools.assertConvexPolygon2DEquals(croppedFootPolygon, partialFootholdModule.getShrunkenFootPolygon(), 1.0e-5);

      for (FramePoint2DReadOnly measuredCoP : measuredCoPs)
         assertTrue(partialFootholdModule.getShrunkenFootPolygon().signedDistance(measuredCoP) < 1e-3);



      // start rotating back the other way, but with the rotation about the front of the foot, which should fail for shrinking the foot since we haven't
      rotationAngularVelocity = new FrameVector3D(soleFrame, 0.0, omegaNorm, 0.0);
      direction = new Vector2D(rotationAngularVelocity);
      direction.normalize();

      // update the sole twist based on the cop location and the angular velocity
      soleTwist.setIncludingFrame(rotationAngularVelocity, new FrameVector3D(soleFrame), pointOfRotation);
      soleFrame.update();

      pointOfRotation.setX(0.05);
      perfectDesiredCoP.setX(0.075);

      for (int increment = 0; increment < historyRequirement; increment++)
      {
         footPitch += omegaNorm * dt;
         solePose.getOrientation().setYawPitchRoll(0.0, footPitch, 0.0);
         soleFrame.update();

         FramePoint2D measuredCoP = new FramePoint2D(soleFrame, direction);
         double scale = InterpolationTools.linearInterpolate(-lineWidth, lineWidth, ((double) increment) / ((double) historyRequirement));
         measuredCoP.scale(scale);
         measuredCoP.add(pointOfRotation.getX(), pointOfRotation.getY());

         measuredCoPs.add(measuredCoP);

         FramePoint2D desiredCoP = new FramePoint2D(soleFrame, direction);
         double desiredScale = InterpolationTools.linearInterpolate(-desiredLineWidth, desiredLineWidth, ((double) increment) / ((double) historyRequirement));
         desiredCoP.scale(desiredScale);
         desiredCoP.add(perfectDesiredCoP.getX(), perfectDesiredCoP.getY());

         // try to estimate the line of rotation which should go through the cop and have the direction of omega
         partialFootholdModule.compute(measuredCoP, desiredCoP);
      }

      assertTrue(partialFootholdModule.isRotating());
      assertEquals(footPitch, 0.0, 1e-5);

      lineEstimate = partialFootholdModule.getLineOfRotation();
      expectedLine = new FrameLine2D(soleFrame);

      expectedLine.getDirection().set(direction);
      expectedLine.getPoint().set(pointOfRotation);
      //      convexPolygonTools.cutPolygonWithLine(expectedLine, croppedFootPolygon, RobotSide.RIGHT);

      EuclidGeometryTestTools.assertLine2DGeometricallyEquals(expectedLine, lineEstimate, 1.0e-5);
      assertFalse(partialFootholdModule.shouldApplyShrunkenFoothold());
      EuclidGeometryTestTools.assertConvexPolygon2DEquals(croppedFootPolygon, partialFootholdModule.getShrunkenFootPolygon(), 1.0e-5);

      for (FramePoint2DReadOnly measuredCoP : measuredCoPs)
         assertTrue(partialFootholdModule.getShrunkenFootPolygon().isPointInside(measuredCoP));

      // we've rotated back to zero, which means we're now pitching about a new line, and should cut
      for (int increment = 0; increment < historyRequirement; increment++)
      {
         footPitch += omegaNorm * dt;
         solePose.getOrientation().setYawPitchRoll(0.0, footPitch, 0.0);
         soleFrame.update();

         FramePoint2D measuredCoP = new FramePoint2D(soleFrame, direction);
         double scale = InterpolationTools.linearInterpolate(-lineWidth, lineWidth, ((double) increment) / ((double) historyRequirement));
         measuredCoP.scale(scale);
         measuredCoP.add(pointOfRotation.getX(), pointOfRotation.getY());

         measuredCoPs.add(measuredCoP);

         FramePoint2D desiredCoP = new FramePoint2D(soleFrame, direction);
         double desiredScale = InterpolationTools.linearInterpolate(-desiredLineWidth, desiredLineWidth, ((double) increment) / ((double) historyRequirement));
         desiredCoP.scale(desiredScale);
         desiredCoP.add(perfectDesiredCoP.getX(), perfectDesiredCoP.getY());

         // try to estimate the line of rotation which should go through the cop and have the direction of omega
         partialFootholdModule.compute(measuredCoP, desiredCoP);
      }

      assertTrue(partialFootholdModule.isRotating());
      assertEquals(footPitch, 0.0, 1e-5);

      lineEstimate = partialFootholdModule.getLineOfRotation();
      expectedLine = new FrameLine2D(soleFrame);

      expectedLine.getDirection().set(direction);
      expectedLine.getPoint().set(pointOfRotation);
      convexPolygonTools.cutPolygonWithLine(expectedLine, croppedFootPolygon, RobotSide.RIGHT);

      EuclidGeometryTestTools.assertLine2DGeometricallyEquals(expectedLine, lineEstimate, 1.0e-5);
      assertFalse(partialFootholdModule.shouldApplyShrunkenFoothold());
      EuclidGeometryTestTools.assertConvexPolygon2DEquals(croppedFootPolygon, partialFootholdModule.getShrunkenFootPolygon(), 1.0e-5);

      for (FramePoint2DReadOnly measuredCoP : measuredCoPs)
         assertTrue(partialFootholdModule.getShrunkenFootPolygon().isPointInside(measuredCoP));
   }

   private class TestSoleFrame extends MovingReferenceFrame
   {
      private final Twist twistRelativeToParent;
      private final Pose3DReadOnly poseRelativeToParent;

      public TestSoleFrame(Twist twistRelativeToParent, Pose3DReadOnly poseRelativeToParent)
      {
         super("TestFrame", ReferenceFrame.getWorldFrame());
         this.twistRelativeToParent = twistRelativeToParent;
         this.poseRelativeToParent = poseRelativeToParent;
      }

      @Override
      protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
      {
         twistRelativeToParentToPack.set(twistRelativeToParent);
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         poseRelativeToParent.get(transformToParent);
      }
   }

   public static FrameConvexPolygon2DBasics createFootPolygon(ReferenceFrame soleFrame, double footLength, double heelWidth, double toeWidth)
   {
      return new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(createFootPolygonPoints(soleFrame, footLength, heelWidth, toeWidth)));
   }

   public static List<? extends FramePoint2DReadOnly> createFootPolygonPoints(ReferenceFrame soleFrame, double footLength, double heelWidth, double toeWidth)
   {
      List<FramePoint2DReadOnly> points = new ArrayList<>();

      points.add(new FramePoint2D(soleFrame, footLength / 2.0, toeWidth / 2.0));
      points.add(new FramePoint2D(soleFrame, footLength / 2.0, -toeWidth / 2.0));
      points.add(new FramePoint2D(soleFrame, -footLength / 2.0, heelWidth / 2.0));
      points.add(new FramePoint2D(soleFrame, -footLength / 2.0, -heelWidth / 2.0));

      return points;
   }
}
