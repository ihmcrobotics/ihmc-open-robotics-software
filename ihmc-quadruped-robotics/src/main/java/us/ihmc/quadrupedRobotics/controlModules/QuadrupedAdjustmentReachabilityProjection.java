package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoInteger;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class QuadrupedAdjustmentReachabilityProjection
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean visualize = true;
   private static final int numberOfVertices = 9;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrantDependentList<List<YoFramePoint2D>> reachabilityVertices = new QuadrantDependentList<>();
   private final QuadrantDependentList<List<YoFramePoint2D>> reachabilityVerticesInWorld = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFrameConvexPolygon2D> reachabilityPolygonsInWorld = new QuadrantDependentList<>();

   private final QuadrantDependentList<YoPlaneContactState> contactStates;

   private final DoubleProvider lengthLimit;
   private final DoubleProvider lengthBackLimit;
   private final DoubleProvider innerLimit;
   private final DoubleProvider outerLimit;

   private final ReferenceFrame bodyZUpFrame;
   private final QuadrupedReferenceFrames referenceFrames;

   public QuadrupedAdjustmentReachabilityProjection(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      contactStates = controllerToolbox.getFootContactStates();

      lengthLimit = new DoubleParameter("MaxReachabilityLength", registry, 0.35);
      lengthBackLimit = new DoubleParameter("MaxReachabilityBackwardLength", registry, -0.3);
      innerLimit = new DoubleParameter("MaxReachabilityWidth", registry, -0.15);
      outerLimit = new DoubleParameter("MinReachabilityWidth", registry, 0.2);

      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getRuntimeEnvironment().getGraphicsListRegistry();

      referenceFrames = controllerToolbox.getReferenceFrames();
      bodyZUpFrame = referenceFrames.getBodyZUpFrame();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         YoInteger yoNumberOfReachabilityVertices = new YoInteger(robotQuadrant.getShortName() + "NumberOfReachabilityVertices", registry);
         yoNumberOfReachabilityVertices.set(numberOfVertices);

         String prefix = robotQuadrant.getShortName() + "Adjustment";

         List<YoFramePoint2D> reachabilityVertices = new ArrayList<>();
         List<YoFramePoint2D> reachabilityVerticesInWorld = new ArrayList<>();
         for (int i = 0; i < yoNumberOfReachabilityVertices.getValue(); i++)
         {
            YoFramePoint2D vertex = new YoFramePoint2D(prefix + "ReachabilityVertex" + i, referenceFrames.getLegAttachmentFrame(robotQuadrant), registry);
            YoFramePoint2D vertexInWorld = new YoFramePoint2D(prefix + "ReachabilityVertexInWorld" + i, worldFrame, registry);
            reachabilityVertices.add(vertex);
            reachabilityVerticesInWorld.add(vertexInWorld);
         }
         YoFrameConvexPolygon2D reachabilityPolygonInWorld = new YoFrameConvexPolygon2D(reachabilityVerticesInWorld, yoNumberOfReachabilityVertices, worldFrame);

         this.reachabilityVertices.put(robotQuadrant, reachabilityVertices);
         this.reachabilityVerticesInWorld.put(robotQuadrant, reachabilityVerticesInWorld);
         this.reachabilityPolygonsInWorld.put(robotQuadrant, reachabilityPolygonInWorld);

         if (yoGraphicsListRegistry != null)
         {
            YoArtifactPolygon reachabilityGraphic = new YoArtifactPolygon(robotQuadrant.getShortName() + "ReachabilityRegionViz", reachabilityPolygonInWorld, Color.BLUE, false);

            reachabilityGraphic.setVisible(visualize);

            yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), reachabilityGraphic);
         }
      }

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactStates.get(robotQuadrant).inContact())
            updateReachabilityPolygonInSupport(robotQuadrant);
         else
            updateReachabilityPolygonInSwing(robotQuadrant);
      }
   }

   public void project(FixedFramePoint3DBasics goalPositionToPack, RobotQuadrant stepQuadrant)
   {
      tempVertex.setIncludingFrame(goalPositionToPack);
      tempVertex.changeFrameAndProjectToXYPlane(worldFrame);

      FrameConvexPolygon2DReadOnly reachabilityPolygonInWorld = reachabilityPolygonsInWorld.get(stepQuadrant);

      if (reachabilityPolygonInWorld.isPointInside(tempVertex))
         return;

      reachabilityPolygonsInWorld.get(stepQuadrant).orthogonalProjection(tempVertex);

      tempVertex.changeFrameAndProjectToXYPlane(goalPositionToPack.getReferenceFrame());
      goalPositionToPack.setX(tempVertex.getX());
      goalPositionToPack.setY(tempVertex.getY());
   }

   public void completedStep(RobotQuadrant stepQuadrant)
   {
      updateReachabilityPolygonInSupport(stepQuadrant);
   }

   private final FramePoint2D tempVertex = new FramePoint2D();
   private void updateReachabilityPolygonInSwing(RobotQuadrant robotQuadrant)
   {
      List<YoFramePoint2D> vertices = reachabilityVertices.get(robotQuadrant);
      List<YoFramePoint2D> verticesInWorld = reachabilityVerticesInWorld.get(robotQuadrant);
      YoFrameConvexPolygon2D polygonInWorld = reachabilityPolygonsInWorld.get(robotQuadrant);

      // create an ellipsoid around the center of the forward and backward reachable limits
      double xRadius = 0.5 * (lengthLimit.getValue() - lengthBackLimit.getValue());
      double yRadius = 0.5 * (outerLimit.getValue() - innerLimit.getValue());
      double centerX = lengthLimit.getValue() - xRadius;
      double centerY = outerLimit.getValue() - yRadius;

      // compute the vertices on the edge of the ellipsoid
      for (int vertexIdx = 0; vertexIdx < vertices.size(); vertexIdx++)
      {
         double angle = 2.0 * Math.PI * vertexIdx / (vertices.size() - 1);
         double x = centerX + xRadius * Math.cos(angle);
         double y = robotQuadrant.getSide().negateIfLeftSide(centerY + yRadius * Math.sin(angle));
         FixedFramePoint2DBasics vertex = vertices.get(vertexIdx);
         vertex.set(x, y);

         tempVertex.setIncludingFrame(vertex);
         tempVertex.changeFrameAndProjectToXYPlane(worldFrame);
         verticesInWorld.get(vertexIdx).set(tempVertex);
      }

      polygonInWorld.notifyVerticesChanged();
      polygonInWorld.update();
   }

   private void updateReachabilityPolygonInSupport(RobotQuadrant supportQuadrant)
   {
      List<YoFramePoint2D> vertices = reachabilityVertices.get(supportQuadrant);
      List<YoFramePoint2D> verticesInWorld = reachabilityVerticesInWorld.get(supportQuadrant);
      YoFrameConvexPolygon2D polygonInWorld = reachabilityPolygonsInWorld.get(supportQuadrant);

      for (int vertexIdx = 0; vertexIdx < vertices.size(); vertexIdx++)
      {
         vertices.get(vertexIdx).setToNaN();
         verticesInWorld.get(vertexIdx).setToNaN();
      }
      polygonInWorld.notifyVerticesChanged();
      polygonInWorld.update();
   }
}
