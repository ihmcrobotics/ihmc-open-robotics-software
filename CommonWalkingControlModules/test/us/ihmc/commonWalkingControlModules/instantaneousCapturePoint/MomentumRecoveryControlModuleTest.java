package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.util.ArrayList;

import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.plotting.Plotter;
import us.ihmc.plotting.PlotterShowHideMenu;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class MomentumRecoveryControlModuleTest
{
   private static final boolean showPlotter = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private MomentumRecoveryControlModule momentumRecoveryControlModule;

   private SideDependentList<RigidBody> feet = new SideDependentList<>();
   private SideDependentList<SixDoFJoint> footJoints = new SideDependentList<>();
   private SideDependentList<ReferenceFrame> ankleFrames = new SideDependentList<>();
   private SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private SideDependentList<FrameConvexPolygon2d> defaultFootPolygons = new SideDependentList<>();

   private BooleanYoVariable allowUpperBodyMomentumInSingleSupport;
   private BooleanYoVariable allowUpperBodyMomentumInDoubleSupport;
   private BooleanYoVariable allowUsingHighMomentumWeight;

   private BooleanYoVariable usingUpperBodyMomentum;
   private BooleanYoVariable usingHighMomentumWeight;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test (timeout = 30000)
   /**
    * This test passes a huge ICP error and a bad ICP to the control module and makes
    * sure recovery gets triggered.
    */
   public void testEnabledDoubleSupport()
   {
      Vector3D leftFootPosition = new Vector3D(0.0, 0.1, 0.0);
      Vector3D rightFootPosition = new Vector3D(0.05, -0.1, 0.0);
      SideDependentList<Vector3D> footPositions = new SideDependentList<Vector3D>(leftFootPosition, rightFootPosition);
      setupTest(footPositions);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame, Double.MAX_VALUE, Double.MAX_VALUE);
      FrameConvexPolygon2d supportPolygon = makeSupportPolygon(true, true);
      FrameVector2d icpError = new FrameVector2d(worldFrame, Double.MAX_VALUE, Double.MAX_VALUE);

      momentumRecoveryControlModule.setSupportSide(null);
      momentumRecoveryControlModule.setICPError(icpError);
      momentumRecoveryControlModule.setCapturePoint(capturePoint);
      momentumRecoveryControlModule.setSupportPolygon(supportPolygon);

      momentumRecoveryControlModule.compute();

      FrameConvexPolygon2d areaToProjectInto = new FrameConvexPolygon2d();
      FrameConvexPolygon2d safeArea = new FrameConvexPolygon2d();
      momentumRecoveryControlModule.getCMPProjectionArea(areaToProjectInto, safeArea);
      boolean useHighMomentumWeight = momentumRecoveryControlModule.getUseHighMomentumWeight();

      assertFalse(safeArea.isEmpty());
      for (int i = 0; i < supportPolygon.getNumberOfVertices(); i++)
      {
         FramePoint2d tmpPoint = new FramePoint2d();
         supportPolygon.getFrameVertex(i, tmpPoint);
         assertTrue(areaToProjectInto.isPointInside(tmpPoint));
      }
      assertTrue(useHighMomentumWeight);

      assertTrue(usingUpperBodyMomentum.getBooleanValue());
      assertTrue(usingHighMomentumWeight.getBooleanValue());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test (timeout = 30000)
   /**
    * This test passes a huge ICP error and a bad ICP to the control module and makes
    * sure recovery gets triggered.
    */
   public void testEnabledSingleSupport()
   {
      Vector3D leftFootPosition = new Vector3D(0.0, 0.1, 0.0);
      Vector3D rightFootPosition = new Vector3D(0.05, -0.1, 0.0);
      SideDependentList<Vector3D> footPositions = new SideDependentList<Vector3D>(leftFootPosition, rightFootPosition);
      setupTest(footPositions);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame, Double.MAX_VALUE, Double.MAX_VALUE);
      FrameConvexPolygon2d supportPolygon = makeSupportPolygon(true, true);
      FrameVector2d icpError = new FrameVector2d(worldFrame, Double.MAX_VALUE, Double.MAX_VALUE);

      momentumRecoveryControlModule.setSupportSide(RobotSide.LEFT);
      momentumRecoveryControlModule.setICPError(icpError);
      momentumRecoveryControlModule.setCapturePoint(capturePoint);
      momentumRecoveryControlModule.setSupportPolygon(supportPolygon);

      momentumRecoveryControlModule.compute();

      FrameConvexPolygon2d areaToProjectInto = new FrameConvexPolygon2d();
      FrameConvexPolygon2d safeArea = new FrameConvexPolygon2d();
      momentumRecoveryControlModule.getCMPProjectionArea(areaToProjectInto, safeArea);
      boolean useHighMomentumWeight = momentumRecoveryControlModule.getUseHighMomentumWeight();

      assertFalse(safeArea.isEmpty());
      for (int i = 0; i < supportPolygon.getNumberOfVertices(); i++)
      {
         FramePoint2d tmpPoint = new FramePoint2d();
         supportPolygon.getFrameVertex(i, tmpPoint);
         assertTrue(areaToProjectInto.isPointInside(tmpPoint));
      }
      assertTrue(useHighMomentumWeight);

      assertTrue(usingUpperBodyMomentum.getBooleanValue());
      assertTrue(usingHighMomentumWeight.getBooleanValue());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test (timeout = 30000)
   /**
    * This test passes a huge ICP error and a bad ICP to the control module but disables momentum
    * recovery. It makes sure the recovery does not get triggered.
    */
   public void testDisabledDoubleSupport()
   {
      Vector3D leftFootPosition = new Vector3D(0.0, 0.1, 0.0);
      Vector3D rightFootPosition = new Vector3D(0.05, -0.1, 0.0);
      SideDependentList<Vector3D> footPositions = new SideDependentList<Vector3D>(leftFootPosition, rightFootPosition);
      setupTest(footPositions);

      allowUpperBodyMomentumInSingleSupport.set(false);
      allowUpperBodyMomentumInDoubleSupport.set(false);
      allowUsingHighMomentumWeight.set(false);

      usingUpperBodyMomentum.set(true);
      usingHighMomentumWeight.set(true);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame, Double.MAX_VALUE, Double.MAX_VALUE);
      FrameConvexPolygon2d supportPolygon = makeSupportPolygon(true, true);
      FrameVector2d icpError = new FrameVector2d(worldFrame, Double.MAX_VALUE, Double.MAX_VALUE);

      momentumRecoveryControlModule.setSupportSide(null);
      momentumRecoveryControlModule.setICPError(icpError);
      momentumRecoveryControlModule.setCapturePoint(capturePoint);
      momentumRecoveryControlModule.setSupportPolygon(supportPolygon);

      momentumRecoveryControlModule.compute();

      FrameConvexPolygon2d areaToProjectInto = new FrameConvexPolygon2d();
      FrameConvexPolygon2d safeArea = new FrameConvexPolygon2d();
      momentumRecoveryControlModule.getCMPProjectionArea(areaToProjectInto, safeArea);
      boolean useHighMomentumWeight = momentumRecoveryControlModule.getUseHighMomentumWeight();

      assertTrue(safeArea.isEmpty());
      assertTrue(areaToProjectInto.epsilonEquals(supportPolygon, 10E-10));
      assertFalse(useHighMomentumWeight);

      assertFalse(usingUpperBodyMomentum.getBooleanValue());
      assertFalse(usingHighMomentumWeight.getBooleanValue());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test (timeout = 30000)
   /**
    * This test passes a huge ICP error and a bad ICP to the control module but disables momentum
    * recovery. It makes sure the recovery does not get triggered.
    */
   public void testDisabledSingleSupport()
   {
      Vector3D leftFootPosition = new Vector3D(0.0, 0.1, 0.0);
      Vector3D rightFootPosition = new Vector3D(0.05, -0.1, 0.0);
      SideDependentList<Vector3D> footPositions = new SideDependentList<Vector3D>(leftFootPosition, rightFootPosition);
      setupTest(footPositions);

      allowUpperBodyMomentumInSingleSupport.set(false);
      allowUpperBodyMomentumInDoubleSupport.set(false);
      allowUsingHighMomentumWeight.set(false);

      usingUpperBodyMomentum.set(true);
      usingHighMomentumWeight.set(true);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame, Double.MAX_VALUE, Double.MAX_VALUE);
      FrameConvexPolygon2d supportPolygon = makeSupportPolygon(true, true);
      FrameVector2d icpError = new FrameVector2d(worldFrame, Double.MAX_VALUE, Double.MAX_VALUE);

      momentumRecoveryControlModule.setSupportSide(RobotSide.LEFT);
      momentumRecoveryControlModule.setICPError(icpError);
      momentumRecoveryControlModule.setCapturePoint(capturePoint);
      momentumRecoveryControlModule.setSupportPolygon(supportPolygon);

      momentumRecoveryControlModule.compute();

      FrameConvexPolygon2d areaToProjectInto = new FrameConvexPolygon2d();
      FrameConvexPolygon2d safeArea = new FrameConvexPolygon2d();
      momentumRecoveryControlModule.getCMPProjectionArea(areaToProjectInto, safeArea);
      boolean useHighMomentumWeight = momentumRecoveryControlModule.getUseHighMomentumWeight();

      assertTrue(safeArea.isEmpty());
      assertTrue(areaToProjectInto.epsilonEquals(supportPolygon, 10E-10));
      assertFalse(useHighMomentumWeight);

      assertFalse(usingUpperBodyMomentum.getBooleanValue());
      assertFalse(usingHighMomentumWeight.getBooleanValue());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test (timeout = 30000)
   public void testLogicDoubleSupportSafe()
   {
      Vector3D leftFootPosition = new Vector3D(0.0, 0.1, 0.0);
      Vector3D rightFootPosition = new Vector3D(0.05, -0.1, 0.0);
      SideDependentList<Vector3D> footPositions = new SideDependentList<Vector3D>(leftFootPosition, rightFootPosition);
      setupTest(footPositions);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame, 0.1, 0.0);
      FrameConvexPolygon2d supportPolygon = makeSupportPolygon(true, true);

      momentumRecoveryControlModule.setSupportSide(null);
      momentumRecoveryControlModule.setICPError(new FrameVector2d(worldFrame));
      momentumRecoveryControlModule.setCapturePoint(capturePoint);
      momentumRecoveryControlModule.setSupportPolygon(supportPolygon);

      momentumRecoveryControlModule.compute();

      FrameConvexPolygon2d areaToProjectInto = new FrameConvexPolygon2d();
      FrameConvexPolygon2d safeArea = new FrameConvexPolygon2d();
      momentumRecoveryControlModule.getCMPProjectionArea(areaToProjectInto, safeArea);

      if (showPlotter)
      {
         class Local {};
         String name = Local.class.getEnclosingMethod().getName();
         showPlotter(yoGraphicsListRegistry, name);

      }

      assertFalse(usingUpperBodyMomentum.getBooleanValue());
      assertFalse(usingHighMomentumWeight.getBooleanValue());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test (timeout = 30000)
   public void testLogicDoubleSupportUnsafe()
   {
      Vector3D leftFootPosition = new Vector3D(0.0, 0.1, 0.0);
      Vector3D rightFootPosition = new Vector3D(0.05, -0.1, 0.0);
      SideDependentList<Vector3D> footPositions = new SideDependentList<Vector3D>(leftFootPosition, rightFootPosition);
      setupTest(footPositions);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame, -0.2, 0.0);
      FrameConvexPolygon2d supportPolygon = makeSupportPolygon(true, true);

      momentumRecoveryControlModule.setSupportSide(null);
      momentumRecoveryControlModule.setICPError(new FrameVector2d(worldFrame));
      momentumRecoveryControlModule.setCapturePoint(capturePoint);
      momentumRecoveryControlModule.setSupportPolygon(supportPolygon);

      momentumRecoveryControlModule.compute();

      FrameConvexPolygon2d areaToProjectInto = new FrameConvexPolygon2d();
      FrameConvexPolygon2d safeArea = new FrameConvexPolygon2d();
      momentumRecoveryControlModule.getCMPProjectionArea(areaToProjectInto, safeArea);

      if (showPlotter)
      {
         class Local {};
         String name = Local.class.getEnclosingMethod().getName();
         showPlotter(yoGraphicsListRegistry, name);

      }

      assertTrue(usingUpperBodyMomentum.getBooleanValue());
      assertTrue(usingHighMomentumWeight.getBooleanValue());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test (timeout = 30000)
   public void testLogicSingleSupportSafe()
   {
      Vector3D leftFootPosition = new Vector3D(0.0, 0.1, 0.0);
      Vector3D rightFootPosition = new Vector3D(0.2, -0.15, 0.0);
      SideDependentList<Vector3D> footPositions = new SideDependentList<Vector3D>(leftFootPosition, rightFootPosition);
      setupTest(footPositions);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame, 0.1, 0.0);

      RobotSide stepSide = RobotSide.RIGHT;
      FrameConvexPolygon2d supportPolygon = makeSupportPolygon(stepSide == RobotSide.RIGHT, stepSide == RobotSide.LEFT);
      FramePose stepPose = new FramePose(worldFrame, rightFootPosition, new Quaternion());
      Footstep footStep = new Footstep(feet.get(stepSide), stepSide, soleFrames.get(stepSide), new PoseReferenceFrame("stepFrame", stepPose));

      momentumRecoveryControlModule.setSupportSide(RobotSide.LEFT);
      momentumRecoveryControlModule.setICPError(new FrameVector2d(worldFrame));
      momentumRecoveryControlModule.setCapturePoint(capturePoint);
      momentumRecoveryControlModule.setSupportPolygon(supportPolygon);
      momentumRecoveryControlModule.setNextFootstep(footStep);

      momentumRecoveryControlModule.compute();

      FrameConvexPolygon2d areaToProjectInto = new FrameConvexPolygon2d();
      FrameConvexPolygon2d safeArea = new FrameConvexPolygon2d();
      momentumRecoveryControlModule.getCMPProjectionArea(areaToProjectInto, safeArea);

      if (showPlotter)
      {
         class Local {};
         String name = Local.class.getEnclosingMethod().getName();
         showPlotter(yoGraphicsListRegistry, name);

      }

      assertFalse(usingUpperBodyMomentum.getBooleanValue());
      assertFalse(usingHighMomentumWeight.getBooleanValue());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test (timeout = 30000)
   public void testLogicSingleSupportUnsafe()
   {
      Vector3D leftFootPosition = new Vector3D(0.0, 0.1, 0.0);
      Vector3D rightFootPosition = new Vector3D(0.2, -0.15, 0.0);
      SideDependentList<Vector3D> footPositions = new SideDependentList<Vector3D>(leftFootPosition, rightFootPosition);
      setupTest(footPositions);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame, 0.35, 0.05);

      RobotSide stepSide = RobotSide.RIGHT;
      FrameConvexPolygon2d supportPolygon = makeSupportPolygon(stepSide == RobotSide.RIGHT, stepSide == RobotSide.LEFT);
      FramePose stepPose = new FramePose(worldFrame, rightFootPosition, new Quaternion());
      Footstep footStep = new Footstep(feet.get(stepSide), stepSide, soleFrames.get(stepSide), new PoseReferenceFrame("stepFrame", stepPose));

      momentumRecoveryControlModule.setSupportSide(RobotSide.LEFT);
      momentumRecoveryControlModule.setICPError(new FrameVector2d(worldFrame));
      momentumRecoveryControlModule.setCapturePoint(capturePoint);
      momentumRecoveryControlModule.setSupportPolygon(supportPolygon);
      momentumRecoveryControlModule.setNextFootstep(footStep);

      momentumRecoveryControlModule.compute();

      FrameConvexPolygon2d areaToProjectInto = new FrameConvexPolygon2d();
      FrameConvexPolygon2d safeArea = new FrameConvexPolygon2d();
      momentumRecoveryControlModule.getCMPProjectionArea(areaToProjectInto, safeArea);

      if (showPlotter)
      {
         class Local {};
         String name = Local.class.getEnclosingMethod().getName();
         showPlotter(yoGraphicsListRegistry, name);

      }

      assertTrue(usingUpperBodyMomentum.getBooleanValue());
      assertTrue(usingHighMomentumWeight.getBooleanValue());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test (timeout = 30000)
   /**
    * This test passes a huge ICP error and a good ICP to the control module and makes
    * sure high weight recovery gets triggered but not upper body momentum.
    */
   public void testHighMomentumWeight()
   {
      Vector3D leftFootPosition = new Vector3D(0.0, 0.1, 0.0);
      Vector3D rightFootPosition = new Vector3D(0.05, -0.1, 0.0);
      SideDependentList<Vector3D> footPositions = new SideDependentList<Vector3D>(leftFootPosition, rightFootPosition);
      setupTest(footPositions);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame);
      FrameConvexPolygon2d supportPolygon = makeSupportPolygon(true, true);
      FrameVector2d icpError = new FrameVector2d(worldFrame, Double.MAX_VALUE, Double.MAX_VALUE);

      momentumRecoveryControlModule.setSupportSide(null);
      momentumRecoveryControlModule.setICPError(icpError);
      momentumRecoveryControlModule.setCapturePoint(capturePoint);
      momentumRecoveryControlModule.setSupportPolygon(supportPolygon);

      momentumRecoveryControlModule.compute();

      FrameConvexPolygon2d areaToProjectInto = new FrameConvexPolygon2d();
      FrameConvexPolygon2d safeArea = new FrameConvexPolygon2d();
      momentumRecoveryControlModule.getCMPProjectionArea(areaToProjectInto, safeArea);
      boolean useHighMomentumWeight = momentumRecoveryControlModule.getUseHighMomentumWeight();

      assertTrue(useHighMomentumWeight);
      assertFalse(usingUpperBodyMomentum.getBooleanValue());
      assertTrue(usingHighMomentumWeight.getBooleanValue());
   }

   private void setupTest(SideDependentList<Vector3D> footPositions)
   {
      RigidBody elevator = new RigidBody("elevator", worldFrame);

      for (RobotSide robotSide : RobotSide.values)
      {
         String prefix = robotSide.getLowerCaseName();
         SixDoFJoint footJoint = new SixDoFJoint(prefix + "FootJoint", elevator, worldFrame);
         RigidBody foot = ScrewTools.addRigidBody(prefix + "Foot", footJoint, new Matrix3D(), 1.0, new Vector3D());
         ReferenceFrame ankleFrame = foot.getBodyFixedFrame();
         ReferenceFrame soleFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent(prefix + "Sole", ankleFrame, new RigidBodyTransform());

         feet.put(robotSide, foot);
         footJoints.put(robotSide, footJoint);
         ankleFrames.put(robotSide, ankleFrame);
         soleFrames.put(robotSide, soleFrame);

         FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d(soleFrame);
         footPolygon.addVertex(new Point2D(0.1, 0.05));
         footPolygon.addVertex(new Point2D(0.1, -0.05));
         footPolygon.addVertex(new Point2D(-0.1, -0.05));
         footPolygon.addVertex(new Point2D(-0.1, 0.05));
         footPolygon.update();
         defaultFootPolygons.put(robotSide, footPolygon);

         footJoint.setPosition(footPositions.get(robotSide));
      }
      elevator.updateFramesRecursively();

      momentumRecoveryControlModule = new MomentumRecoveryControlModule(defaultFootPolygons, 0.1, registry, yoGraphicsListRegistry);
      allowUpperBodyMomentumInSingleSupport = (BooleanYoVariable) registry.getVariable("allowUpperBodyMomentumInSingleSupport");
      allowUpperBodyMomentumInDoubleSupport = (BooleanYoVariable) registry.getVariable("allowUpperBodyMomentumInDoubleSupport");
      allowUsingHighMomentumWeight = (BooleanYoVariable) registry.getVariable("allowUsingHighMomentumWeight");

      allowUpperBodyMomentumInSingleSupport.set(true);
      allowUpperBodyMomentumInDoubleSupport.set(true);
      allowUsingHighMomentumWeight.set(true);

      usingUpperBodyMomentum = (BooleanYoVariable) registry.getVariable("usingUpperBodyMomentum");
      usingHighMomentumWeight = (BooleanYoVariable) registry.getVariable("usingHighMomentumWeight");

      ArtifactList artifacts = new ArtifactList(getClass().getSimpleName());
      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2d footPolygonInWorld = new FrameConvexPolygon2d();
         footPolygonInWorld.setIncludingFrame(defaultFootPolygons.get(robotSide));
         footPolygonInWorld.changeFrameAndProjectToXYPlane(worldFrame);

         String prefix = robotSide.getLowerCaseName();
         String Prefix = robotSide.getCamelCaseNameForMiddleOfExpression();
         YoFrameConvexPolygon2d yoFootPolygon = new YoFrameConvexPolygon2d(prefix + "FootPolygon", worldFrame, 10, registry);
         artifacts.add(new YoArtifactPolygon(Prefix + " Foot Polygon", yoFootPolygon, Color.BLACK, false, 1));
         yoFootPolygon.setFrameConvexPolygon2d(footPolygonInWorld);
      }
      yoGraphicsListRegistry.registerArtifactList(artifacts);
   }

   private FrameConvexPolygon2d makeSupportPolygon(boolean leftFootContact, boolean rightFootContact)
   {
      FrameConvexPolygon2d support = new FrameConvexPolygon2d(worldFrame);
      for (RobotSide robotSide : RobotSide.values)
      {
         if (robotSide == RobotSide.LEFT && !leftFootContact)
            continue;
         if (robotSide == RobotSide.RIGHT && !rightFootContact)
            continue;

         FrameConvexPolygon2d footPolygon = defaultFootPolygons.get(robotSide);
         FramePoint2d tempPoint = new FramePoint2d();
         for (int i = 0; i < footPolygon.getNumberOfVertices(); i++)
         {
            footPolygon.getFrameVertex(i, tempPoint);
            tempPoint.changeFrameAndProjectToXYPlane(worldFrame);
            support.addVertex(tempPoint);
         }
      }
      support.update();
      return support;
   }

   private void showPlotter(YoGraphicsListRegistry yoGraphicsListRegistry, String windowName)
   {
      Plotter plotter = new Plotter();
      plotter.setViewRange(1.0);

      ArrayList<ArtifactList> artifactLists = new ArrayList<>();
      yoGraphicsListRegistry.getRegisteredArtifactLists(artifactLists);
      for (ArtifactList artifactList : artifactLists)
      {
         artifactList.setVisible(true);
      }

      JFrame frame = new JFrame(windowName);
      Dimension preferredSize = new Dimension(1000, 650);
      frame.setPreferredSize(preferredSize);

      JCheckBox doneBox = new JCheckBox("Done");
      PlotterShowHideMenu plotterShowHideMenu = new PlotterShowHideMenu(plotter);
      plotter.addArtifactsChangedListener(plotterShowHideMenu);

      JPanel menuFrame = new JPanel();
      menuFrame.add(plotterShowHideMenu, BorderLayout.LINE_START);
      JScrollPane scrollPane = new JScrollPane(menuFrame);

      frame.add(scrollPane, BorderLayout.EAST);
      frame.add(doneBox, BorderLayout.SOUTH);
      frame.add(plotter.getJPanel(), BorderLayout.CENTER);

      frame.setSize(preferredSize);
      frame.setVisible(true);

      yoGraphicsListRegistry.addArtifactListsToPlotter(plotter);

      while (!doneBox.isSelected())
      {
         try
         {
            Thread.sleep(100);
         }
         catch (InterruptedException ex)
         {
         }
      }

      frame.setVisible(false);
      frame.dispose();
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(MomentumRecoveryControlModule.class, MomentumRecoveryControlModuleTest.class);
   }

}
