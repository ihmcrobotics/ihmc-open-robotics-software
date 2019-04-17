package us.ihmc.footstepPlanning.ui.viewers;

import javafx.animation.AnimationTimer;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.SettableFootstepPlannerParameters;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette2D;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

public class NodeCheckerRenderer extends AnimationTimer
{
   private final AtomicReference<Boolean> nodeCheckerEnabled;
   private final AtomicReference<PlanarRegionsList> planarRegionsReference;
   private final AtomicReference<Point3D> footPositionReference;
   private final AtomicReference<Quaternion> footOrientationReference;
   private final AtomicReference<RobotSide> initialSupportSideReference;

   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final SettableFootstepPlannerParameters parameters = new SettableFootstepPlannerParameters(new DefaultFootstepPlanningParameters());
   private final SimplePlanarRegionFootstepNodeSnapper snapper;

   private final FootstepNodeChecker nodeChecker;

   private final MeshView meshView = new MeshView();
   private final JavaFXMultiColorMeshBuilder meshBuilder;

   private static final Color ghostFootstepColor = Color.color(0.2, 0.2, 0.2, 0.2);

   public NodeCheckerRenderer(Messager messager, RobotContactPointParameters<RobotSide> robotContactPointParameters)
   {
      nodeCheckerEnabled = messager.createInput(EnableNodeChecking, false);
      planarRegionsReference = messager.createInput(PlanarRegionDataTopic);
      footPositionReference = messager.createInput(NodeCheckingPosition);
      footOrientationReference = messager.createInput(NodeCheckingOrientation, new Quaternion());
      initialSupportSideReference = messager.createInput(InitialSupportSideTopic, RobotSide.LEFT);

      TextureColorPalette2D colorPalette = new TextureColorPalette2D();
      colorPalette.setHueBrightnessBased(0.9);
      meshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

      if(robotContactPointParameters == null)
      {
         footPolygons = PlannerTools.createDefaultFootPolygons();
      }
      else
      {
         SegmentDependentList<RobotSide, ArrayList<Point2D>> controllerFootGroundContactPoints = robotContactPointParameters
               .getControllerFootGroundContactPoints();

         footPolygons = new SideDependentList<>();
         for(RobotSide robotSide : RobotSide.values)
         {
            ConvexPolygon2D footPolygon = new ConvexPolygon2D();
            controllerFootGroundContactPoints.get(robotSide).forEach(footPolygon::addVertex);
            footPolygon.update();
            footPolygons.put(robotSide, footPolygon);
         }
      }

      snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);

      FootstepNodeBodyCollisionDetector collisionDetector = new FootstepNodeBodyCollisionDetector(parameters);
      SnapBasedNodeChecker snapBasedNodeChecker = new SnapBasedNodeChecker(parameters, footPolygons, snapper);
      BodyCollisionNodeChecker bodyCollisionNodeChecker = new BodyCollisionNodeChecker(collisionDetector, parameters, snapper);
      PlanarRegionBaseOfCliffAvoider cliffAvoider = new PlanarRegionBaseOfCliffAvoider(parameters, snapper, footPolygons);
      nodeChecker = new FootstepNodeCheckerOfCheckers(Arrays.asList(snapBasedNodeChecker, bodyCollisionNodeChecker, cliffAvoider));

      messager.registerTopicListener(FootstepPlannerMessagerAPI.PlannerParametersTopic, parameters::set);
   }

   @Override
   public void handle(long now)
   {
      if(nodeCheckerEnabled == null)
         return;

      if(!nodeCheckerEnabled.get())
         return;

      Point3D footPosition = footPositionReference.get();
      double footOrientation = footOrientationReference.get().getYaw();
      PlanarRegionsList planarRegionsList = planarRegionsReference.get();

      if(footPosition == null || planarRegionsList == null)
         return;

      FootstepNode node = new FootstepNode(footPosition.getX(), footPosition.getY(), footOrientation, initialSupportSideReference.get());
      snapper.setPlanarRegions(planarRegionsList);
      nodeChecker.setPlanarRegions(planarRegionsList);

      FootstepNodeSnapData snapData = snapper.snapFootstepNode(node);
      boolean isValid = nodeChecker.isNodeValid(node, null);

      processFootMesh(node, snapData, isValid);
   }

   private void processFootMesh(FootstepNode node, FootstepNodeSnapData snapData, boolean valid)
   {
      meshBuilder.clear();

      RigidBodyTransform planarTransformToWorld = new RigidBodyTransform();
      FootstepNodeTools.getNodeTransform(node, planarTransformToWorld);

      RigidBodyTransform snappedTransformToWorld = new RigidBodyTransform();
      ConvexPolygon2D foothold = snapData.getCroppedFoothold();

      try
      {
         FootstepNodeTools.getSnappedNodeTransform(node, snapData.getSnapTransform(), snappedTransformToWorld);
      }
      catch(NotARotationMatrixException e)
      {
         return;
      }

      snappedTransformToWorld.appendTranslation(0.0, 0.0, 0.01);
      planarTransformToWorld.setTranslationZ(snappedTransformToWorld.getTranslationZ() + 0.1);

      Color regionColor = valid ? Color.GREEN : Color.RED;
      regionColor = Color.hsb(regionColor.getHue(), 0.9, 1.0);

      Point2D[] vertices = new Point2D[foothold.getNumberOfVertices()];
      for (int j = 0; j < vertices.length; j++)
      {
         vertices[j] = new Point2D(foothold.getVertex(j));
      }

      meshBuilder.addMultiLine(snappedTransformToWorld, vertices, 0.01, regionColor, true);
      meshBuilder.addPolygon(snappedTransformToWorld, foothold, regionColor);

      // TODO add mesh of planar footstep

      meshView.setOpacity(0.9);
      meshView.setMesh(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
   }

   public Node getRoot()
   {
      return meshView;
   }
}
