package us.ihmc.quadrupedRobotics.planning.chooser.swingLegChooser;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicEllipsoid;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.controller.position.states.QuadrupedPositionBasedCrawlControllerParameters;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.CommonQuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.geometry.QuadrupedGeometryTools;
import us.ihmc.quadrupedRobotics.geometry.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.shapes.FrameEllipsoid3d;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class LongestFeasibleStepChooser implements NextSwingLegChooser
{
   private static final double ZERO_THRESHOLD = 1e-4;
   private static final double MYSTERIOUS_SCALAR = 1.0;
   private final QuadrupedPositionBasedCrawlControllerParameters quadrupedControllerParameters;
   private CommonQuadrupedReferenceFrames commonQuadrupedReferenceFrames;
   private final QuadrantDependentList<FrameEllipsoid3d> actualFootstepWorkspaces = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoGraphicEllipsoid> footstepWorkspaceYoEllipsoids = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFramePoint> footstepWorkspaceCenterFramePoints = new QuadrantDependentList<>();
   private RobotQuadrant greatestDistanceFeasibleFootstep;

   private final FramePoint temporaryCentroid = new FramePoint(ReferenceFrame.getWorldFrame());

   public LongestFeasibleStepChooser(QuadrupedPositionBasedCrawlControllerParameters quadrupedControllerParameters, CommonQuadrupedReferenceFrames commonQuadrupedReferenceFrames, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.quadrupedControllerParameters = quadrupedControllerParameters;
      this.commonQuadrupedReferenceFrames = commonQuadrupedReferenceFrames;
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         actualFootstepWorkspaces.set(robotQuadrant, new FrameEllipsoid3d(commonQuadrupedReferenceFrames.getHipPitchFrame(robotQuadrant), 0.0, 0.0, 0.0));
         YoFramePoint footstepWorkspaceCenterFramePoint = new YoFramePoint(robotQuadrant + "FootstepWorkspaceFramePoint", ReferenceFrame.getWorldFrame(), registry);
         footstepWorkspaceCenterFramePoints.set(robotQuadrant, footstepWorkspaceCenterFramePoint);
         YoFrameOrientation footstepWorkspaceOrientation = new YoFrameOrientation(robotQuadrant + "FootstepWorkspaceOrientation", ReferenceFrame.getWorldFrame(), registry);
         footstepWorkspaceYoEllipsoids.set(robotQuadrant, new YoGraphicEllipsoid(robotQuadrant + "YoEllipsoid", footstepWorkspaceCenterFramePoint, footstepWorkspaceOrientation, YoAppearance.Blue(), new Vector3d()));
         yoGraphicsListRegistry.registerYoGraphic(robotQuadrant + "YoEllipsoidReg", footstepWorkspaceYoEllipsoids.get(robotQuadrant));
         footstepWorkspaceYoEllipsoids.get(robotQuadrant).showGraphicObject();
      }
   }
   
   /**
    * Do all the setup stuff.
    */
   private void updateState()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         // Calculate footstep workspace ellipse size
         QuadrupedGeometryTools.updateFootstepWorkspace(robotQuadrant, actualFootstepWorkspaces.get(robotQuadrant), commonQuadrupedReferenceFrames);
         
         // Update workspace center YoFramePoints
         FramePoint workspaceCenterPoint = footstepWorkspaceCenterFramePoints.get(robotQuadrant).getFrameTuple();
         double hipHeight = ReferenceFrame.getWorldFrame().getTransformToDesiredFrame(commonQuadrupedReferenceFrames.getHipPitchFrame(robotQuadrant)).getM23();
         
         double x = ReferenceFrame.getWorldFrame().getTransformToDesiredFrame(commonQuadrupedReferenceFrames.getHipPitchFrame(robotQuadrant)).getM03();
         System.out.println("X: " + x);
         
         workspaceCenterPoint.changeFrame(commonQuadrupedReferenceFrames.getHipPitchFrame(robotQuadrant));
         workspaceCenterPoint.set(0.0, 0.0, hipHeight);
         footstepWorkspaceCenterFramePoints.get(robotQuadrant).setAndMatchFrame(workspaceCenterPoint);
         
         // Update YoGraphicEllipsoids to footstep workspace
         Vector3d radiiToPack = new Vector3d();
         actualFootstepWorkspaces.get(robotQuadrant).getEllipsoid3d().getRadii(radiiToPack);
         radiiToPack.setZ(0.001);
         footstepWorkspaceYoEllipsoids.get(robotQuadrant).setRadii(radiiToPack);
         footstepWorkspaceYoEllipsoids.get(robotQuadrant).update();
      }
   }
   
   /**
    * Do the interesting part.
    */
   private RobotQuadrant doTheCalculations(FrameVector desiredVelocity, double desiredYawRate, RobotQuadrant lastStepFoot, QuadrupedSupportPolygon actualSupportPolygon)
   {
      if (desiredVelocity.getX() < ZERO_THRESHOLD && desiredVelocity.getY() < ZERO_THRESHOLD && desiredYawRate < ZERO_THRESHOLD)
      {
         // So stepping in place looks good and all legs step.
         return lastStepFoot.getNextRegularGaitSwingQuadrant();
      }
      else // Robot is actually moving in a direction or yawing
      {
         // Construct polygon such that the legs conform to a right rectangle with stance height and width, centered at the centroid and pointing toward nominal yaw
         QuadrupedSupportPolygon actualPerfectPolygon = new QuadrupedSupportPolygon();
         double nominalYaw = actualSupportPolygon.getNominalYaw();
         double perfectStanceYOffset = quadrupedControllerParameters.getStanceLength() / 2;
         double perfectStanceXOffset = quadrupedControllerParameters.getStanceWidth() / 2;
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            actualSupportPolygon.getCentroid(temporaryCentroid);
            temporaryCentroid.add(robotQuadrant.getSide().negateIfLeftSide(perfectStanceXOffset), robotQuadrant.getEnd().negateIfHindEnd(perfectStanceYOffset), 0.0);
            actualPerfectPolygon.setFootstep(robotQuadrant, temporaryCentroid);
         }
         actualPerfectPolygon.yawAboutCentroid(nominalYaw);
         
         // Construct a polygon moved and yawed by some scalar times the desired velocity and yaw
         QuadrupedSupportPolygon desiredPerfectPolygon = new QuadrupedSupportPolygon(actualPerfectPolygon);
         Vector3d stepTranslation = new Vector3d(desiredVelocity.getVector());
         stepTranslation.scale(MYSTERIOUS_SCALAR);
         desiredPerfectPolygon.translate(stepTranslation);
         desiredPerfectPolygon.yawAboutCentroid(MYSTERIOUS_SCALAR * desiredYawRate);
         
         // Check each desired perfect foot and store the furthest feasible from actual
         greatestDistanceFeasibleFootstep = null;
         double greatestDistance = -1.0;
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
         {
            FramePoint desiredPerfectFoot = desiredPerfectPolygon.getFootstep(robotQuadrant);
            desiredPerfectFoot.changeFrame(commonQuadrupedReferenceFrames.getHipPitchFrame(robotQuadrant));
            if (actualFootstepWorkspaces.get(robotQuadrant).isInsideOrOnSurface(desiredPerfectFoot, 1e-7))
            {
               double distance = desiredPerfectFoot.distance(actualSupportPolygon.getFootstep(robotQuadrant));
               if (distance > greatestDistance)
               {
                  greatestDistance = distance;
                  greatestDistanceFeasibleFootstep = robotQuadrant;
               }
            }
         }
         
         if (greatestDistance > -1.0)
         {
            return greatestDistanceFeasibleFootstep;
         }
         
//         throw new RuntimeException(desiredPerfectPolygon + " not feasible.");
         return null;
      }
   }
   
   @Override
   public RobotQuadrant chooseNextSwingLeg(QuadrupedSupportPolygon supportPolygon, RobotQuadrant lastStepFoot, FrameVector desiredVelocity, double desiredYawRate)
   {
      updateState();
      return doTheCalculations(desiredVelocity, desiredYawRate, lastStepFoot, supportPolygon);
   }
}
