package us.ihmc.quadrupedFootstepPlanning.ui.viewers;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import javafx.animation.AnimationTimer;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette2D;
import us.ihmc.messager.Messager;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersBasics;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.SimplePlanarRegionPawNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNodeTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeChecking.PawNodeTransitionChecker;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeChecking.PawNodeTransitionCheckerOfCheckers;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeChecking.SnapBasedPawNodeTransitionChecker;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.DefaultPawStepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;

public class PawNodeCheckerRenderer extends AnimationTimer
{
   private final AtomicReference<Boolean> nodeCheckerEnabled;
   private final AtomicReference<Boolean> checkNodeUsingPoseBetweenFeet;
   private final AtomicReference<PlanarRegionsList> planarRegionsReference;
   private final AtomicReference<Point3D> footPositionReference;
   private final AtomicReference<Quaternion> footOrientationReference;
   private final AtomicReference<RobotQuadrant> initialSupportQuadrantReference;

   private final PawStepPlannerParametersBasics parameters = new DefaultPawStepPlannerParameters();
   private final QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();
   private final SimplePlanarRegionPawNodeSnapper snapper;

   private final PawNodeTransitionChecker nodeChecker;

   private final MeshView meshView = new MeshView();
   private final JavaFXMultiColorMeshBuilder meshBuilder;

   private static final Color ghostFootstepColor = Color.color(0.2, 0.2, 0.2, 0.2);

   public PawNodeCheckerRenderer(Messager messager)
   {
      nodeCheckerEnabled = messager.createInput(PawStepPlannerMessagerAPI.EnableNodeChecking, false);
      planarRegionsReference = messager.createInput(PawStepPlannerMessagerAPI.PlanarRegionDataTopic);
      footPositionReference = messager.createInput(PawStepPlannerMessagerAPI.NodeCheckingPosition);
      footOrientationReference = messager.createInput(PawStepPlannerMessagerAPI.NodeCheckingOrientation, new Quaternion());
      initialSupportQuadrantReference = messager.createInput(PawStepPlannerMessagerAPI.InitialSupportQuadrantTopic, RobotQuadrant.FRONT_LEFT);
      checkNodeUsingPoseBetweenFeet = messager.createInput(PawStepPlannerMessagerAPI.NodeCheckingPoseBetweenFeetTopic, false);

      TextureColorPalette2D colorPalette = new TextureColorPalette2D();
      colorPalette.setHueBrightnessBased(0.9);
      meshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

      snapper = new SimplePlanarRegionPawNodeSnapper(parameters, parameters::getProjectInsideDistance,
                                                     parameters::getProjectInsideUsingConvexHull, true);

      SnapBasedPawNodeTransitionChecker snapBasedNodeChecker = new SnapBasedPawNodeTransitionChecker(parameters, snapper);
      nodeChecker = new PawNodeTransitionCheckerOfCheckers(Arrays.asList(snapBasedNodeChecker));

      messager.registerTopicListener(PawStepPlannerMessagerAPI.PlannerParametersTopic, parameters::set);
      messager.registerTopicListener(PawStepPlannerMessagerAPI.XGaitSettingsTopic, xGaitSettings::set);
   }

   @Override
   public void handle(long now)
   {
      if(nodeCheckerEnabled == null)
         return;

      if(!nodeCheckerEnabled.get())
         return;

      Point3D footPosition = footPositionReference.get();
      Orientation3DReadOnly footOrientation = footOrientationReference.get();
      PlanarRegionsList planarRegionsList = planarRegionsReference.get();

      if(footPosition == null || planarRegionsList == null)
         return;

      FramePoint2D frontLeftPosition = new FramePoint2D();
      FramePoint2D frontRightPosition = new FramePoint2D();
      FramePoint2D hindLeftPosition = new FramePoint2D();
      FramePoint2D hindRightPosition = new FramePoint2D();


      if (!checkNodeUsingPoseBetweenFeet.get())
      {
         switch (initialSupportQuadrantReference.get())
         {
         case FRONT_LEFT:
            frontLeftPosition.set(footPosition.getX(), footPosition.getY());

            Vector2D toFrontRight = new Vector2D(0.0, -xGaitSettings.getStanceWidth());
            Vector2D toHindLeft = new Vector2D(-xGaitSettings.getStanceLength(), 0.0);
            Vector2D toHindRight = new Vector2D(-xGaitSettings.getStanceLength(), -xGaitSettings.getStanceWidth());

            footOrientation.transform(toFrontRight);
            footOrientation.transform(toHindLeft);
            footOrientation.transform(toHindRight);

            frontRightPosition.set(frontLeftPosition);
            hindLeftPosition.set(frontLeftPosition);
            hindRightPosition.set(frontLeftPosition);

            frontRightPosition.add(toFrontRight);
            hindLeftPosition.add(toHindLeft);
            hindRightPosition.add(toHindRight);
            break;
         case FRONT_RIGHT:
            frontRightPosition.set(footPosition.getX(), footPosition.getY());

            Vector2D toFrontLeft = new Vector2D(0.0, xGaitSettings.getStanceWidth());
            toHindRight = new Vector2D(-xGaitSettings.getStanceLength(), 0.0);
            toHindLeft = new Vector2D(-xGaitSettings.getStanceLength(), xGaitSettings.getStanceWidth());

            footOrientation.transform(toFrontLeft);
            footOrientation.transform(toHindLeft);
            footOrientation.transform(toHindRight);

            frontLeftPosition.set(frontRightPosition);
            hindLeftPosition.set(frontRightPosition);
            hindRightPosition.set(frontRightPosition);

            frontLeftPosition.add(toFrontLeft);
            hindLeftPosition.add(toHindLeft);
            hindRightPosition.add(toHindRight);
            break;
         case HIND_LEFT:
            hindLeftPosition.set(footPosition.getX(), footPosition.getY());

            toFrontLeft = new Vector2D(xGaitSettings.getStanceLength(), 0.0);
            toFrontRight = new Vector2D(xGaitSettings.getStanceLength(), -xGaitSettings.getStanceWidth());
            toHindRight = new Vector2D(0.0, -xGaitSettings.getStanceWidth());

            footOrientation.transform(toFrontLeft);
            footOrientation.transform(toFrontRight);
            footOrientation.transform(toHindRight);

            frontLeftPosition.set(hindLeftPosition);
            frontRightPosition.set(hindLeftPosition);
            hindRightPosition.set(hindLeftPosition);

            frontLeftPosition.add(toFrontLeft);
            frontRightPosition.add(toFrontRight);
            hindRightPosition.add(toHindRight);
            break;
         case HIND_RIGHT:
            hindRightPosition.set(footPosition.getX(), footPosition.getY());

            toFrontLeft = new Vector2D(xGaitSettings.getStanceLength(), xGaitSettings.getStanceWidth());
            toFrontRight = new Vector2D(xGaitSettings.getStanceLength(), 0.0);
            toHindLeft = new Vector2D(0.0, xGaitSettings.getStanceWidth());

            footOrientation.transform(toFrontLeft);
            footOrientation.transform(toFrontRight);
            footOrientation.transform(toHindLeft);

            frontLeftPosition.set(hindRightPosition);
            frontRightPosition.set(hindRightPosition);
            hindLeftPosition.set(hindRightPosition);

            frontLeftPosition.add(toFrontLeft);
            frontRightPosition.add(toFrontRight);
            hindLeftPosition.add(toHindLeft);
            break;
         }
      }
      else
      {
         Vector2D toFoot = new Vector2D(xGaitSettings.getStanceLength(), xGaitSettings.getStanceWidth());
         footOrientation.transform(toFoot);

         frontLeftPosition.set(footPosition.getX(), footPosition.getY());
         frontRightPosition.set(footPosition.getX(), footPosition.getY());
         hindLeftPosition.set(footPosition.getX(), footPosition.getY());
         hindRightPosition.set(footPosition.getX(), footPosition.getY());

         frontLeftPosition.add(toFoot.getX(), toFoot.getY());
         frontRightPosition.add(toFoot.getX(), -toFoot.getY());
         hindLeftPosition.add(-toFoot.getX(), toFoot.getY());
         hindRightPosition.add(-toFoot.getX(), -toFoot.getY());
      }

      double yaw = PawNode.computeNominalYaw(frontLeftPosition.getX(), frontLeftPosition.getY(), frontRightPosition.getX(), frontRightPosition.getY(),
                                             hindLeftPosition.getX(), hindLeftPosition.getY(), hindRightPosition.getX(), hindRightPosition.getY());
      PawNode node = new PawNode(initialSupportQuadrantReference.get(), frontLeftPosition, frontRightPosition, hindLeftPosition, hindRightPosition,
                                 yaw, xGaitSettings.getStanceLength(), xGaitSettings.getStanceWidth());
      snapper.setPlanarRegions(planarRegionsList);
      nodeChecker.setPlanarRegions(planarRegionsList);

      PawNodeSnapData snapData = snapper.snapPawNode(node);
      boolean isValid = nodeChecker.isNodeValid(node, null);

      processFootMesh(node, snapData, isValid);
   }

   private void processFootMesh(PawNode node, PawNodeSnapData snapData, boolean valid)
   {
      meshBuilder.clear();

      RigidBodyTransform planarTransformToWorld = new RigidBodyTransform();
      PawNodeTools.getNodeTransformToWorld(node.getMovingQuadrant(), node, planarTransformToWorld);

      RigidBodyTransform snappedTransformToWorld = new RigidBodyTransform();

      try
      {
         PawNodeTools.getSnappedNodeTransformToWorld(node.getMovingQuadrant(), node, snapData.getSnapTransform(), snappedTransformToWorld);
      }
      catch(NotARotationMatrixException e)
      {
         return;
      }

      snappedTransformToWorld.appendTranslation(0.0, 0.0, 0.01);
      planarTransformToWorld.setTranslationZ(snappedTransformToWorld.getTranslationZ() + 0.1);

      Color regionColor = valid ? Color.GREEN : Color.RED;
      regionColor = Color.hsb(regionColor.getHue(), 0.9, 1.0);

      meshBuilder.addSphere(0.1, snappedTransformToWorld.getTranslationVector(), regionColor);

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
