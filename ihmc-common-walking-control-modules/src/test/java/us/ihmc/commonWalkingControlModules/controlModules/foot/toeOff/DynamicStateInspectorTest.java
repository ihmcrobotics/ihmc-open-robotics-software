package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer.defaultFeetColors;
import static us.ihmc.robotics.Assert.*;

public class DynamicStateInspectorTest
{
   private static final double epsilon = 1e-5;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double footLength = 0.2;
   private static final double footWidth = 0.1;

   private FrameConvexPolygon2D leftPolygon;
   private FrameConvexPolygon2D rightPolygon;
   private FrameConvexPolygon2D onToesPolygon;
   private YoRegistry registry;
   private FramePoint2D toePosition;
   private FramePoint2D desiredICP;
   private FramePoint2D currentICP;

   @BeforeEach
   public void setup()
   {
      registry = new YoRegistry("test");

      leftPolygon = createFootPolygon(footLength, footWidth);
      rightPolygon = createFootPolygon(footLength, footWidth);
      onToesPolygon = new FrameConvexPolygon2D();
      desiredICP = new FramePoint2D();
      currentICP = new FramePoint2D();

      toePosition = new FramePoint2D();
   }

   @AfterEach
   public void tearDown()
   {
      leftPolygon = null;
      rightPolygon = null;
   }

   @Test
   public void testLeftStep()
   {
      DynamicStateInspectorParameters parameters = new DynamicStateInspectorParameters(registry);
      DynamicStateInspector inspector = new DynamicStateInspector(registry);

      FramePose3D leftFootPose = new FramePose3D();
      FramePose3D rightFootPose = new FramePose3D();

      double stepLength = 0.6;
      double stepWidth = 0.2;

      leftFootPose.getPosition().set(stepLength, 0.5 * stepWidth, 0.0);
      rightFootPose.getPosition().set(0.0, -0.5 * stepWidth, 0.0);

      leftPolygon.translate(leftFootPose.getX(), leftFootPose.getY());
      rightPolygon.translate(rightFootPose.getX(), rightFootPose.getY());

      toePosition.setIncludingFrame(rightFootPose.getPosition());
      toePosition.addX(0.5 * footLength);

      onToesPolygon.addVertices(leftPolygon);
      onToesPolygon.addVertex(toePosition);
      onToesPolygon.update();

      inspector.setPolygons(leftPolygon, rightPolygon, onToesPolygon);

      desiredICP.interpolate(new FramePoint2D(rightFootPose.getPosition()), new FramePoint2D(leftFootPose.getPosition()), 0.75);

      // test really inside
      currentICP.set(desiredICP);
      currentICP.subX(0.05);

      inspector.checkICPLocations(parameters, RobotSide.RIGHT, leftFootPose, desiredICP, currentICP, toePosition);

      double expectedICPX = 0.75 * stepLength - 0.05;
      double expectedICPY = 0.25 * stepWidth;

      int outsideVertexIdx = leftPolygon.lineOfSightEndIndex(toePosition);
      int insideVertexIdx = leftPolygon.lineOfSightStartIndex(toePosition);

      assertEquals(expectedICPX, currentICP.getX(), epsilon);
      assertEquals(expectedICPY, currentICP.getY(), epsilon);

      assertTrue(onToesPolygon.isPointInside(currentICP));
      assertTrue(onToesPolygon.isPointInside(desiredICP));

      Line2D errorLine = new Line2D();
      Vector2D errorDirection = new Vector2D();
      errorDirection.sub(currentICP, desiredICP);
      errorLine.set(currentICP, errorDirection);

      double expectedDistanceAlongErrorToOutside = -intersectionDistanceBetweenRay2DAndLineSegment2D(currentICP,
                                                                                                     errorLine.getPoint(),
                                                                                                     errorLine.getDirection(),
                                                                                                     toePosition,
                                                                                                     leftPolygon.getVertex(outsideVertexIdx));
      double expectedDistanceAlongErrorToInside = -intersectionDistanceBetweenRay2DAndLineSegment2D(currentICP,
                                                                                                    errorLine.getPoint(),
                                                                                                    errorLine.getDirection(),
                                                                                                    toePosition,
                                                                                                    leftPolygon.getVertex(insideVertexIdx));

      assertEquals(-EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(currentICP, toePosition, leftPolygon.getVertex(outsideVertexIdx)),
                   inspector.getCurrentOrthogonalDistanceToOutsideEdge(),
                   epsilon);
      assertEquals(-EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(desiredICP, toePosition, leftPolygon.getVertex(outsideVertexIdx)),
                   inspector.getDesiredOrthogonalDistanceToOutsideEdge(),
                   epsilon);
      assertEquals(-EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(currentICP, toePosition, leftPolygon.getVertex(insideVertexIdx)),
                   inspector.getCurrentOrthogonalDistanceToInsideEdge(),
                   epsilon);
      assertEquals(-EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(desiredICP, toePosition, leftPolygon.getVertex(insideVertexIdx)),
                   inspector.getDesiredOrthogonalDistanceToInsideEdge(),
                   epsilon);
      assertEquals(expectedICPY + 0.5 * stepWidth, inspector.getLateralDistanceOfDesiredICPInside(), epsilon);
      assertEquals(expectedICPY + 0.5 * stepWidth, inspector.getLateralDistanceOfCurrentICPInside(), epsilon);
      assertEquals(toePosition.distanceSquared(desiredICP), inspector.getDistanceSquaredOfDesiredICPFromToe(), epsilon);
      assertEquals(toePosition.distanceSquared(currentICP), inspector.getDistanceSquaredOfCurrentICPFromToe(), epsilon);
      assertEquals(expectedDistanceAlongErrorToOutside, inspector.getDistanceAlongErrorToOutsideEdge(), epsilon);
      assertEquals(Double.NEGATIVE_INFINITY, inspector.getDistanceAlongErrorToInsideEdge(), epsilon);

      assertFalse(inspector.isCurrentICPIsPastTheHeel());
      assertTrue(inspector.isCurrentICPFarEnoughFromTheToe());
      assertTrue(inspector.isDesiredICPFarEnoughFromTheToe());
      assertTrue(inspector.isCurrentICPFarEnoughInside());
      assertTrue(inspector.isDesiredICPFarEnoughInside());

//      visualize();

      // test falling behind outside outside support
      currentICP.set(desiredICP);
      currentICP.subX(0.15);

      inspector.checkICPLocations(parameters, RobotSide.RIGHT, leftFootPose, desiredICP, currentICP, toePosition);

      expectedICPX = 0.75 * stepLength - 0.15;
      expectedICPY = 0.25 * stepWidth;

      assertEquals(expectedICPX, currentICP.getX(), epsilon);
      assertEquals(expectedICPY, currentICP.getY(), epsilon);

      expectedDistanceAlongErrorToOutside = intersectionDistanceBetweenRay2DAndLineSegment2D(currentICP,
                                                                                             errorLine.getPoint(),
                                                                                             errorLine.getDirection(),
                                                                                             toePosition,
                                                                                             leftPolygon.getVertex(outsideVertexIdx));
      expectedDistanceAlongErrorToInside = -intersectionDistanceBetweenRay2DAndLineSegment2D(currentICP,
                                                                                             errorLine.getPoint(),
                                                                                             errorLine.getDirection(),
                                                                                             toePosition,
                                                                                             leftPolygon.getVertex(insideVertexIdx));

      assertTrue(!onToesPolygon.isPointInside(currentICP));
      assertTrue(onToesPolygon.isPointInside(desiredICP));

      assertEquals(EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(currentICP, toePosition, leftPolygon.getVertex(outsideVertexIdx)),
                   inspector.getCurrentOrthogonalDistanceToOutsideEdge(),
                   epsilon);
      assertEquals(-EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(desiredICP, toePosition, leftPolygon.getVertex(outsideVertexIdx)),
                   inspector.getDesiredOrthogonalDistanceToOutsideEdge(),
                   epsilon);
      assertEquals(-EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(currentICP, toePosition, leftPolygon.getVertex(insideVertexIdx)),
                   inspector.getCurrentOrthogonalDistanceToInsideEdge(),
                   epsilon);
      assertEquals(-EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(desiredICP, toePosition, leftPolygon.getVertex(insideVertexIdx)),
                   inspector.getDesiredOrthogonalDistanceToInsideEdge(),
                   epsilon);
      assertEquals(expectedICPY + 0.5 * stepWidth, inspector.getLateralDistanceOfDesiredICPInside(), epsilon);
      assertEquals(expectedICPY + 0.5 * stepWidth, inspector.getLateralDistanceOfCurrentICPInside(), epsilon);
      assertEquals(toePosition.distanceSquared(desiredICP), inspector.getDistanceSquaredOfDesiredICPFromToe(), epsilon);
      assertEquals(toePosition.distanceSquared(currentICP), inspector.getDistanceSquaredOfCurrentICPFromToe(), epsilon);
      assertEquals(expectedDistanceAlongErrorToOutside, inspector.getDistanceAlongErrorToOutsideEdge(), epsilon);
      assertEquals(Double.NEGATIVE_INFINITY, inspector.getDistanceAlongErrorToInsideEdge(), epsilon);

      assertFalse(inspector.isCurrentICPIsPastTheHeel());
      assertTrue(inspector.isCurrentICPFarEnoughFromTheToe());
      assertTrue(inspector.isDesiredICPFarEnoughFromTheToe());
      assertTrue(inspector.isCurrentICPFarEnoughInside());
      assertTrue(inspector.isDesiredICPFarEnoughInside());

      // test error towards inside
      currentICP.set(desiredICP);
      currentICP.subY(0.05);

      inspector.checkICPLocations(parameters, RobotSide.RIGHT, leftFootPose, desiredICP, currentICP, toePosition);
      visualize();

      expectedICPX = 0.75 * stepLength - 0.15;
      expectedICPY = 0.25 * stepWidth;

      assertEquals(expectedICPX, currentICP.getX(), epsilon);
      assertEquals(expectedICPY, currentICP.getY(), epsilon);

      expectedDistanceAlongErrorToOutside = intersectionDistanceBetweenRay2DAndLineSegment2D(currentICP,
                                                                                             errorLine.getPoint(),
                                                                                             errorLine.getDirection(),
                                                                                             toePosition,
                                                                                             leftPolygon.getVertex(outsideVertexIdx));
      expectedDistanceAlongErrorToInside = -intersectionDistanceBetweenRay2DAndLineSegment2D(currentICP,
                                                                                             errorLine.getPoint(),
                                                                                             errorLine.getDirection(),
                                                                                             toePosition,
                                                                                             leftPolygon.getVertex(insideVertexIdx));

      assertTrue(!onToesPolygon.isPointInside(currentICP));
      assertTrue(onToesPolygon.isPointInside(desiredICP));

      assertEquals(EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(currentICP, toePosition, leftPolygon.getVertex(outsideVertexIdx)),
                   inspector.getCurrentOrthogonalDistanceToOutsideEdge(),
                   epsilon);
      assertEquals(-EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(desiredICP, toePosition, leftPolygon.getVertex(outsideVertexIdx)),
                   inspector.getDesiredOrthogonalDistanceToOutsideEdge(),
                   epsilon);
      assertEquals(-EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(currentICP, toePosition, leftPolygon.getVertex(insideVertexIdx)),
                   inspector.getCurrentOrthogonalDistanceToInsideEdge(),
                   epsilon);
      assertEquals(-EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(desiredICP, toePosition, leftPolygon.getVertex(insideVertexIdx)),
                   inspector.getDesiredOrthogonalDistanceToInsideEdge(),
                   epsilon);
      assertEquals(expectedICPY + 0.5 * stepWidth, inspector.getLateralDistanceOfDesiredICPInside(), epsilon);
      assertEquals(expectedICPY + 0.5 * stepWidth, inspector.getLateralDistanceOfCurrentICPInside(), epsilon);
      assertEquals(toePosition.distanceSquared(desiredICP), inspector.getDistanceSquaredOfDesiredICPFromToe(), epsilon);
      assertEquals(toePosition.distanceSquared(currentICP), inspector.getDistanceSquaredOfCurrentICPFromToe(), epsilon);
      assertEquals(expectedDistanceAlongErrorToOutside, inspector.getDistanceAlongErrorToOutsideEdge(), epsilon);
      assertEquals(Double.NEGATIVE_INFINITY, inspector.getDistanceAlongErrorToInsideEdge(), epsilon);

      assertFalse(inspector.isCurrentICPIsPastTheHeel());
      assertTrue(inspector.isCurrentICPFarEnoughFromTheToe());
      assertTrue(inspector.isDesiredICPFarEnoughFromTheToe());
      assertTrue(inspector.isCurrentICPFarEnoughInside());
      assertTrue(inspector.isDesiredICPFarEnoughInside());

      visualize();
   }

   @Test
   public void test20220206_164237_Nadia_HeuristicICPController_StepsAndFallCrop01()
   {
      DynamicStateInspectorParameters parameters = new DynamicStateInspectorParameters(registry);
      DynamicStateInspector inspector = new DynamicStateInspector(registry);

      leftPolygon.clear(worldFrame);
      rightPolygon.clear(worldFrame);

      leftPolygon.addVertex(-0.21496, -0.1422);
      leftPolygon.addVertex(-0.2109, -0.0471);
      leftPolygon.addVertex(0.0039, -0.0565);
      leftPolygon.addVertex(-0.0002, -0.1514);
      leftPolygon.update();

      rightPolygon.addVertex(-0.4375, -0.2933);
      rightPolygon.addVertex(-00.2314, -0.2321);
      rightPolygon.addVertex(-0.2046, -0.3231);
      rightPolygon.addVertex(-0.4106, -0.3843);
      rightPolygon.update();

      toePosition.setIncludingFrame(worldFrame, -0.2188, -0.2754);

      onToesPolygon.clear(worldFrame);
      onToesPolygon.addVertices(leftPolygon);
      onToesPolygon.addVertex(toePosition);
      onToesPolygon.update();

      desiredICP.setIncludingFrame(worldFrame, -0.0952,  -0.1387);
      currentICP.setIncludingFrame(worldFrame, -0.1589, -0.247);

      FramePose3D leftFootPose = new FramePose3D();
      leftFootPose.getPosition().set(-0.1057, -0.0995, 0.1598);
      leftFootPose.getOrientation().set(0.006, -0.009, -0.0226, 0.9997);

      inspector.setPolygons(leftPolygon, rightPolygon, onToesPolygon);
      inspector.checkICPLocations(parameters, RobotSide.RIGHT, leftFootPose, desiredICP, currentICP, toePosition);

      assertFalse(inspector.areDynamicsOkForToeOff());

      visualize();

   }

   @Test
   public void test20220206_164237_Nadia_HeuristicICPController_StepsAndFallCrop02_6_99()
   {
      DynamicStateInspectorParameters parameters = new DynamicStateInspectorParameters(registry);
      DynamicStateInspector inspector = new DynamicStateInspector(registry);

      leftPolygon.clear(worldFrame);
      rightPolygon.clear(worldFrame);

      leftPolygon.addVertex(0.4577, -0.2276);
      leftPolygon.addVertex(0.6708, -0.1995);
      leftPolygon.addVertex(0.6832, -0.2936);
      leftPolygon.addVertex(0.4701, -0.3217);
      leftPolygon.update();

      rightPolygon.addVertex(0.7722, -0.4052);
      rightPolygon.addVertex(0.9845, -0.3713);
      rightPolygon.addVertex(0.9995, -0.4651);
      rightPolygon.addVertex(0.7872, -0.499);
      rightPolygon.update();

      toePosition.setIncludingFrame(worldFrame, 0.6762, -0.2465);

      onToesPolygon.clear(worldFrame);
      onToesPolygon.addVertices(rightPolygon);
      onToesPolygon.addVertex(toePosition);
      onToesPolygon.update();

      desiredICP.setIncludingFrame(worldFrame, 0.7908,  -0.3611);
      currentICP.setIncludingFrame(worldFrame, 0.771, -0.3625);

      FramePose3D rightFootPose = new FramePose3D();
      rightFootPose.getPosition().set(0.8859, -0.4352, 0.3122);
      rightFootPose.getOrientation().set(-0.007, -0.0065, 0.079, 0.9968);

      inspector.setPolygons(rightPolygon, leftPolygon, onToesPolygon);
      inspector.checkICPLocations(parameters, RobotSide.LEFT, rightFootPose, desiredICP, currentICP, toePosition);

      assertTrue(inspector.areDynamicsOkForToeOff());

      visualize();

   }

   @Test
   public void test20220206_164237_Nadia_HeuristicICPController_StepsAndFallCrop02_8_863()
   {
      DynamicStateInspectorParameters parameters = new DynamicStateInspectorParameters(registry);
      DynamicStateInspector inspector = new DynamicStateInspector(registry);

      leftPolygon.clear(worldFrame);
      rightPolygon.clear(worldFrame);

      leftPolygon.addVertex(0.4642, -0.4675);
      leftPolygon.addVertex(0.6779, -0.4443);
      leftPolygon.addVertex(0.6882, -0.5388);
      leftPolygon.addVertex(0.4744, -0.562);
      leftPolygon.update();

      rightPolygon.addVertex(0.2512, -0.7194);
      rightPolygon.addVertex(0.4662, -0.7136);
      rightPolygon.addVertex(0.4687, -0.8086);
      rightPolygon.addVertex(0.2538, -0.8143);
      rightPolygon.update();

      toePosition.setIncludingFrame(worldFrame, 0.4675, -0.7611);

      onToesPolygon.clear(worldFrame);
      onToesPolygon.addVertices(leftPolygon);
      onToesPolygon.addVertex(toePosition);
      onToesPolygon.update();

      desiredICP.setIncludingFrame(worldFrame, 0.4901,  -0.5979);
      currentICP.setIncludingFrame(worldFrame, 0.4808, -0.6091);

      FramePose3D leftFootPose = new FramePose3D();
      leftFootPose.getPosition().set(0.5762, -0.5031, 0.8193);
      leftFootPose.getOrientation().set(0.0007, -0.0034, 0.054, 0.9985);

      inspector.setPolygons(leftPolygon, rightPolygon, onToesPolygon);
      inspector.checkICPLocations(parameters, RobotSide.LEFT, leftFootPose, desiredICP, currentICP, toePosition);

      assertFalse(inspector.areDynamicsOkForToeOff());

      visualize();

   }

   @Test
   public void test20220208_110756_Nadia_HeuristicICPController_StepsAndFallCrop02_58_471()
   {
      DynamicStateInspectorParameters parameters = new DynamicStateInspectorParameters(registry);
      DynamicStateInspector inspector = new DynamicStateInspector(registry);

      leftPolygon.clear(worldFrame);
      rightPolygon.clear(worldFrame);

      leftPolygon.addVertex(8.567, 0.5775);
      leftPolygon.addVertex(8.5894, 0.6696);
      leftPolygon.addVertex(8.7984, 0.6189);
      leftPolygon.addVertex(8.776, 0.5267);
      leftPolygon.update();

      rightPolygon.addVertex(8.247, 0.3501);
      rightPolygon.addVertex(8.422, 0.4748);
      rightPolygon.addVertex(8.477, 0.3974);
      rightPolygon.addVertex(8.3019, 0.2727);
      rightPolygon.update();

      toePosition.setIncludingFrame(worldFrame, 8.4521, 0.4323);

      onToesPolygon.clear(worldFrame);
      onToesPolygon.addVertices(leftPolygon);
      onToesPolygon.addVertex(toePosition);
      onToesPolygon.update();

      desiredICP.setIncludingFrame(worldFrame, 8.689,  0.5687);
      currentICP.setIncludingFrame(worldFrame, 8.6432, 0.4951);

      FramePose3D leftFootPose = new FramePose3D();
      leftFootPose.getPosition().set(8.6826, 0.598, 0.8839);
      leftFootPose.getOrientation().set(0.0286, -0.0021, -0.1191, 0.9925);

      inspector.setPolygons(leftPolygon, rightPolygon, onToesPolygon);
      inspector.checkICPLocations(parameters, RobotSide.LEFT, leftFootPose, desiredICP, currentICP, toePosition);

      assertFalse(inspector.areDynamicsOkForToeOff());

      visualize();
   }

   @Test
   public void test20220208_110756_Nadia_HeuristicICPController_StepsAndFallCrop08_24_175()
   {
      DynamicStateInspectorParameters parameters = new DynamicStateInspectorParameters(registry);
      DynamicStateInspector inspector = new DynamicStateInspector(registry);

      leftPolygon.clear(worldFrame);
      rightPolygon.clear(worldFrame);

      leftPolygon.addVertex(80.8065, 2.0266);
      leftPolygon.addVertex(80.8454, 2.1133);
      leftPolygon.addVertex(81.0416, 2.0253);
      leftPolygon.addVertex(81.0027, 1.9387);
      leftPolygon.update();

      rightPolygon.addVertex(80.5009, 1.9292);
      rightPolygon.addVertex(80.7087, 1.9844);
      rightPolygon.addVertex(80.7331, 1.8927);
      rightPolygon.addVertex(80.5254, 1.8375);
      rightPolygon.update();

      toePosition.setIncludingFrame(worldFrame, 80.7219, 1.9349);

      onToesPolygon.clear(worldFrame);
      onToesPolygon.addVertices(leftPolygon);
      onToesPolygon.addVertex(toePosition);
      onToesPolygon.update();

      desiredICP.setIncludingFrame(worldFrame, 80.8725,  1.9723);
      currentICP.setIncludingFrame(worldFrame, 80.8416, 1.9398);

      FramePose3D leftFootPose = new FramePose3D();
      leftFootPose.getPosition().set(80.924, 2.0259, 3.4787);
      leftFootPose.getOrientation().set(-0.0048, -0.0063, -0.21, 0.9777);

      inspector.setPolygons(leftPolygon, rightPolygon, onToesPolygon);
      inspector.checkICPLocations(parameters, RobotSide.LEFT, leftFootPose, desiredICP, currentICP, toePosition);

      assertFalse(inspector.areDynamicsOkForToeOff());

      visualize();
   }

   @Test
   public void testLeftStepGrid()
   {
      DynamicStateInspectorParameters parameters = new DynamicStateInspectorParameters(registry);
      DynamicStateInspector inspector = new DynamicStateInspector(registry);

      FramePose3D leftFootPose = new FramePose3D();
      FramePose3D rightFootPose = new FramePose3D();

      double stepLength = 0.6;
      double stepWidth = 0.2;

      leftFootPose.getPosition().set(stepLength, 0.5 * stepWidth, 0.0);
      rightFootPose.getPosition().set(0.0, -0.5 * stepWidth, 0.0);

      leftPolygon.translate(leftFootPose.getX(), leftFootPose.getY());
      rightPolygon.translate(rightFootPose.getX(), rightFootPose.getY());

      toePosition.setIncludingFrame(rightFootPose.getPosition());
      toePosition.addX(0.5 * footLength);

      onToesPolygon.addVertices(leftPolygon);
      onToesPolygon.addVertex(toePosition);
      onToesPolygon.update();

      inspector.setPolygons(leftPolygon, rightPolygon, onToesPolygon);

      desiredICP = new FramePoint2D();
      desiredICP.interpolate(new FramePoint2D(rightFootPose.getPosition()), new FramePoint2D(leftFootPose.getPosition()), 0.75);


      visualizeGrid(parameters, inspector, stepLength, stepWidth, leftFootPose);
   }

   private static FrameConvexPolygon2D createFootPolygon(double length, double width)
   {
      FrameConvexPolygon2D polygon2D = new FrameConvexPolygon2D();
      polygon2D.addVertex(0.5 * length, 0.5 * width);
      polygon2D.addVertex(0.5 * length, -0.5 * width);
      polygon2D.addVertex(-0.5 * length, -0.5 * width);
      polygon2D.addVertex(-0.5 * length, 0.5 * width);
      polygon2D.update();

      return polygon2D;
   }


   private void visualize()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"));

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      YoFrameConvexPolygon2D yoToeOffPolygonViz = new YoFrameConvexPolygon2D("toeOffPolygon", "", worldFrame, 8, registry);
      YoFrameConvexPolygon2D yoSupportPolygonViz = new YoFrameConvexPolygon2D("combinedPolygon", "", worldFrame, 8, registry);
      YoFrameConvexPolygon2D yoLeftFootPolygonViz = new YoFrameConvexPolygon2D("LeftFootPolygon", "", worldFrame, 4, registry);
      YoFrameConvexPolygon2D yoRightFootPolygonViz = new YoFrameConvexPolygon2D("RightFootPolygon", "", worldFrame, 4, registry);
      YoFramePoint2D yoDesiredICP = new YoFramePoint2D("desiredICP", worldFrame, registry);
      YoFramePoint2D yoCurrentICP = new YoFramePoint2D("currentICP", worldFrame, registry);

      YoArtifactPolygon leftFootViz = new YoArtifactPolygon("Left Foot Polygon", yoLeftFootPolygonViz, defaultFeetColors.get(RobotSide.LEFT), false);
      YoArtifactPolygon rightFootViz = new YoArtifactPolygon("Right Foot Polygon", yoRightFootPolygonViz, defaultFeetColors.get(RobotSide.RIGHT), false);
      YoArtifactPolygon supportPolygonArtifact = new YoArtifactPolygon("Combined Polygon", yoSupportPolygonViz, Color.pink, false);
      YoArtifactPolygon toeOffPolygonArtifact = new YoArtifactPolygon("Toe Off Polygon", yoToeOffPolygonViz, Color.RED, false);
      YoGraphicPosition desiredICPArtifact = new YoGraphicPosition("Desired ICP", yoDesiredICP, 0.01, YoAppearance.Yellow(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition currentICPArtifact = new YoGraphicPosition("Current ICP", yoCurrentICP, 0.01, YoAppearance.Blue(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);

      graphicsListRegistry.registerArtifact("test", leftFootViz);
      graphicsListRegistry.registerArtifact("test", rightFootViz);
      graphicsListRegistry.registerArtifact("test", supportPolygonArtifact);
      graphicsListRegistry.registerArtifact("test", toeOffPolygonArtifact);
      graphicsListRegistry.registerArtifact("test", desiredICPArtifact.createArtifact());
      graphicsListRegistry.registerArtifact("test", currentICPArtifact.createArtifact());

      yoLeftFootPolygonViz.set(leftPolygon);
      yoRightFootPolygonViz.set(rightPolygon);
      yoSupportPolygonViz.addVertices(yoLeftFootPolygonViz);
      yoSupportPolygonViz.addVertices(yoRightFootPolygonViz);
      yoSupportPolygonViz.update();
      yoToeOffPolygonViz.set(onToesPolygon);
      yoDesiredICP.set(desiredICP);
      yoCurrentICP.set(currentICP);


      scs.getRootRegistry().addChild(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
      plotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      plotterFactory.createOverheadPlotter();

      scs.startOnAThread();


      scs.tickAndUpdate();

      ThreadTools.sleepForever();
   }

   private void visualizeGrid(DynamicStateInspectorParameters parameters, DynamicStateInspector inspector, double stepLength, double stepWidth, FramePose3D leadingFootPose)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"));

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      YoFrameConvexPolygon2D yoToeOffPolygonViz = new YoFrameConvexPolygon2D("toeOffPolygon", "", worldFrame, 8, registry);
      YoFrameConvexPolygon2D yoSupportPolygonViz = new YoFrameConvexPolygon2D("combinedPolygon", "", worldFrame, 8, registry);
      YoFrameConvexPolygon2D yoLeftFootPolygonViz = new YoFrameConvexPolygon2D("LeftFootPolygon", "", worldFrame, 4, registry);
      YoFrameConvexPolygon2D yoRightFootPolygonViz = new YoFrameConvexPolygon2D("RightFootPolygon", "", worldFrame, 4, registry);
      YoFramePoint2D yoDesiredICP = new YoFramePoint2D("desiredICP", worldFrame, registry);

      YoArtifactPolygon leftFootViz = new YoArtifactPolygon("Left Foot Polygon", yoLeftFootPolygonViz, defaultFeetColors.get(RobotSide.LEFT), false);
      YoArtifactPolygon rightFootViz = new YoArtifactPolygon("Right Foot Polygon", yoRightFootPolygonViz, defaultFeetColors.get(RobotSide.RIGHT), false);
      YoArtifactPolygon supportPolygonArtifact = new YoArtifactPolygon("Combined Polygon", yoSupportPolygonViz, Color.pink, false);
      YoArtifactPolygon toeOffPolygonArtifact = new YoArtifactPolygon("Toe Off Polygon", yoToeOffPolygonViz, Color.RED, false);
      YoGraphicPosition desiredICPArtifact = new YoGraphicPosition("Desired ICP", yoDesiredICP, 0.02, YoAppearance.Blue(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);

      graphicsListRegistry.registerArtifact("test", leftFootViz);
      graphicsListRegistry.registerArtifact("test", rightFootViz);
      graphicsListRegistry.registerArtifact("test", supportPolygonArtifact);
      graphicsListRegistry.registerArtifact("test", toeOffPolygonArtifact);
      graphicsListRegistry.registerArtifact("test", desiredICPArtifact.createArtifact());

      yoLeftFootPolygonViz.set(leftPolygon);
      yoRightFootPolygonViz.set(rightPolygon);
      yoSupportPolygonViz.addVertices(yoLeftFootPolygonViz);
      yoSupportPolygonViz.addVertices(yoRightFootPolygonViz);
      yoSupportPolygonViz.update();
      yoToeOffPolygonViz.set(onToesPolygon);
      yoDesiredICP.set(desiredICP);


      double gridWidth = stepWidth + footWidth;
      double gridHeight = stepLength + footLength;
      double gridRez = 0.025;
      int widthTicks = (int) (gridWidth / gridRez);
      int heightTicks = (int) (gridHeight / gridRez);
      int points = widthTicks * heightTicks;

      List<YoFramePoint2D> validPoints = new ArrayList<>();
      List<YoFramePoint2D> invalidPoints = new ArrayList<>();

      for (int i = 0; i < points; i++)
      {
         YoFramePoint2D validPoint = new YoFramePoint2D("validPoint" + i, worldFrame, registry);
         YoFramePoint2D invalidPoint = new YoFramePoint2D("invalidPoint" + i, worldFrame, registry);
         validPoint.setToNaN();
         invalidPoint.setToNaN();
         validPoints.add(validPoint);
         invalidPoints.add(invalidPoint);

         validPoint.setToNaN();
         invalidPoint.setToNaN();

         YoGraphicPosition validViz = new YoGraphicPosition("valid point" + i, validPoint, gridRez / 5, YoAppearance.Yellow(), YoGraphicPosition.GraphicType.SOLID_BALL);
         YoGraphicPosition invalidViz = new YoGraphicPosition("invalid point" + i, invalidPoint, gridRez / 5, YoAppearance.Red(), YoGraphicPosition.GraphicType.SOLID_BALL);

         graphicsListRegistry.registerArtifact("test", validViz.createArtifact());
         graphicsListRegistry.registerArtifact("test", invalidViz.createArtifact());
      }


      scs.getRootRegistry().addChild(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
      plotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      plotterFactory.createOverheadPlotter();

      scs.startOnAThread();

      YoVariableChangedListener changedListener = v -> updateListener(stepLength, stepWidth, parameters, inspector, leadingFootPose, validPoints, invalidPoints, scs);
      parameters.attachParameterChangeListener(changedListener);

      updateListener(stepLength, stepWidth, parameters, inspector, leadingFootPose, validPoints, invalidPoints, scs);





      scs.tickAndUpdate();

      ThreadTools.sleepForever();
   }

   private void updateListener(double stepLength, double stepWidth, DynamicStateInspectorParameters parameters, DynamicStateInspector inspector,
                               FramePose3D leadingFootPose, List<YoFramePoint2D> validPoints,
                               List<YoFramePoint2D> invalidPoints, SimulationConstructionSet scs)
   {
      double gridWidth = stepWidth + footWidth;
      double gridHeight = stepLength + footLength;
      double gridRez = 0.025;
      int widthTicks = (int) (gridWidth / gridRez);
      int heightTicks = (int) (gridHeight / gridRez);
      int points = widthTicks * heightTicks;


      double topLeftX = 0.5 * stepLength + 0.5 * gridHeight;
      double topLeftY = 0.5 * gridWidth;

      int currentValidIdx = 0;
      int currentInvalidIdx = 0;

      for (int widthIdx = 0; widthIdx < widthTicks; widthIdx++)

      {
         for (int heightIdx = 0; heightIdx < heightTicks; heightIdx++)
         {
            FramePoint2D currentICP = new FramePoint2D(worldFrame, topLeftX - heightIdx * gridRez, topLeftY - widthIdx * gridRez);
            inspector.checkICPLocations(parameters, RobotSide.RIGHT, leadingFootPose, desiredICP, currentICP, toePosition);

            if (inspector.areDynamicsOkForToeOff())
            {
               validPoints.get(currentValidIdx).set(currentICP);
               currentValidIdx++;
            }
            else
            {
               invalidPoints.get(currentInvalidIdx).set(currentICP);
               currentInvalidIdx++;
            }
         }
      }
            for (int i = currentValidIdx; i < validPoints.size(); i++)
               validPoints.get(i).setToNaN();
            for (int i = currentInvalidIdx; i < invalidPoints.size(); i++)
               invalidPoints.get(i).setToNaN();

      scs.tickAndUpdate();
   }

   private static double intersectionDistanceBetweenRay2DAndLineSegment2D(Point2DReadOnly currentICP,
                                                                          Point2DReadOnly rayStart,
                                                                          Vector2DReadOnly rayDirection,
                                                                          Point2DReadOnly startPoint,
                                                                          Point2DReadOnly endPoint)
   {
      Point2DReadOnly intersection = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayStart, rayDirection, startPoint, endPoint);
      if (intersection == null)
         return Double.POSITIVE_INFINITY;

      return intersection.distance(currentICP);
   }
}
