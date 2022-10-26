package us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher;

import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.SteppableRegionsProvider;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.List;
import java.util.function.Function;

public class BipedalSupportPlanarRegionCalculator implements SteppableRegionsProvider
{
   private static final int LEFT_FOOT_INDEX = 0;
   private static final int RIGHT_FOOT_INDEX = 1;
   private static final int CONVEX_HULL_INDEX = 2;

   private final RecyclingArrayList<PlanarRegion> planarRegions = new RecyclingArrayList<>(PlanarRegion::new);
   private final PlanarRegionCommand[] supportRegions = new PlanarRegionCommand[3];

   private final HumanoidReferenceFrames referenceFrames;
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final SideDependentList<RecyclingArrayList<FramePoint2D>> scaledContactPointList = new SideDependentList<>(new RecyclingArrayList<>(FramePoint2D::new),
                                                                                                                      new RecyclingArrayList<>(FramePoint2D::new));

   private final Function<RobotSide, Boolean> contactStateProvider;

   private final PlanarRegionCommand leftRegion = new PlanarRegionCommand();
   private final PlanarRegionCommand rightRegion = new PlanarRegionCommand();
   private final SideDependentList<PlanarRegionCommand> footRegions = new SideDependentList<>(leftRegion, rightRegion);
   private final PlanarRegionCommand combinedRegion = new PlanarRegionCommand();

   private final FramePoint2D tempPoint = new FramePoint2D();

   public BipedalSupportPlanarRegionCalculator(HumanoidReferenceFrames referenceFrames,
                                               SideDependentList<ConvexPolygon2D> footPolygons,
                                               Function<RobotSide, Boolean> contactStateProviders)
   {
      this.contactStateProvider = contactStateProviders;
      this.footPolygons = footPolygons;
      this.referenceFrames = referenceFrames;
   }

   public void initializeEmptyRegions()
   {
      supportRegions[LEFT_FOOT_INDEX] = null;
      supportRegions[RIGHT_FOOT_INDEX] = null;
      supportRegions[CONVEX_HULL_INDEX] = null;
   }

   public void calculateSupportRegions(double scaleFactor)
   {
      initializeEmptyRegions();

      for (RobotSide robotSide : RobotSide.values)
      {
         scaledContactPointList.get(robotSide).clear();
         for (int i = 0; i < footPolygons.get(robotSide).getNumberOfVertices(); i++)
         {
            Point2DReadOnly contactPoint = footPolygons.get(robotSide).getVertex(i);
            scaledContactPointList.get(robotSide).add().setAndScale(scaleFactor, contactPoint);
         }
      }

      if (feetAreInSamePlane(contactStateProvider))
      {
         ReferenceFrame leftSoleFrame = referenceFrames.getSoleFrame(RobotSide.LEFT);

         combinedRegion.clear();
         combinedRegion.setRegionProperties(CONVEX_HULL_INDEX, leftSoleFrame.getTransformToRoot());
         ConvexPolygon2D convexHull = combinedRegion.getConvexPolygons().add();
         convexHull.clear();

         for (RobotSide robotSide : RobotSide.values)
         {
            for (int i = 0; i < scaledContactPointList.get(robotSide).size(); i++)
            {
               tempPoint.setIncludingFrame(scaledContactPointList.get(robotSide).get(i));
               tempPoint.changeFrameAndProjectToXYPlane(leftSoleFrame);
               convexHull.addVertex(tempPoint);
            }
         }
         convexHull.update();
         for (int i = 0; i < convexHull.getNumberOfVertices(); i++)
            combinedRegion.getConcaveHullsVertices().add().set(convexHull.getVertex(i));

         supportRegions[CONVEX_HULL_INDEX] = combinedRegion;
      }
      else
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            if (contactStateProvider.apply(robotSide))
            {
               List<FramePoint2D> contactPoints = scaledContactPointList.get(robotSide);
               RigidBodyTransform transformToWorld = referenceFrames.getSoleFrame(robotSide).getTransformToRoot();

               PlanarRegionCommand footRegion = footRegions.get(robotSide);
               footRegion.clear();
               footRegion.setRegionProperties(CONVEX_HULL_INDEX, transformToWorld);
               ConvexPolygon2D convexHull = combinedRegion.getConvexPolygons().add();
               convexHull.clear();

               for (int i = 0; i < contactPoints.size(); i++)
                  convexHull.addVertex(contactPoints.get(i));
               convexHull.update();
               for (int i = 0; i < convexHull.getNumberOfVertices(); i++)
                  footRegion.getConcaveHullsVertices().add().set(convexHull.getVertex(i));

               supportRegions[robotSide.ordinal()] = footRegion;
            }
         }
      }
   }

   private boolean feetAreInSamePlane(Function<RobotSide, Boolean> contactStateProvider)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (!contactStateProvider.apply(robotSide))
         {
            return false;
         }
      }
      ReferenceFrame leftSoleFrame = referenceFrames.getSoleFrame(RobotSide.LEFT);
      ReferenceFrame rightSoleFrame = referenceFrames.getSoleFrame(RobotSide.RIGHT);
      RigidBodyTransform relativeSoleTransform = leftSoleFrame.getTransformToDesiredFrame(rightSoleFrame);
      RotationMatrixReadOnly relativeOrientation = relativeSoleTransform.getRotation();

      double rotationEpsilon = Math.toRadians(3.0);
      double translationEpsilon = 0.02;
      return Math.abs(relativeOrientation.getPitch()) < rotationEpsilon && Math.abs(relativeOrientation.getRoll()) < rotationEpsilon
             && Math.abs(relativeSoleTransform.getTranslationZ()) < translationEpsilon;
   }

   public PlanarRegionCommand[] getSupportRegions()
   {
      for (int i = 0; i < 3; i++)
      {
         if (supportRegions[i] != null)
            supportRegions[i].setPlanarRegionId(i);
      }
      return supportRegions;
   }

   @Override
   public void consume(PlanarRegionsListCommand command)
   {
   }

   @Override
   public List<PlanarRegion> getSteppableRegions()
   {
      planarRegions.clear();
      for (PlanarRegionCommand command : getSupportRegions())
      {
         if (command == null)
            continue;
         PlanarRegion planarRegion = planarRegions.add();
         command.getPlanarRegion(planarRegion);
         planarRegions.add(planarRegion);
      }
      return planarRegions;
   }
}
