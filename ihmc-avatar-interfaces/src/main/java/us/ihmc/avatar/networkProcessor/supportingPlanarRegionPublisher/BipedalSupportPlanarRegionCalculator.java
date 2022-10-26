package us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class BipedalSupportPlanarRegionCalculator
{
   private static final int LEFT_FOOT_INDEX = 0;
   private static final int RIGHT_FOOT_INDEX = 1;
   private static final int CONVEX_HULL_INDEX = 2;

   private final PlanarRegionCommand[] supportRegions = new PlanarRegionCommand[3];

   private final SideDependentList<ContactablePlaneBody> contactableFeet;
   private final SideDependentList<RecyclingArrayList<FramePoint2D>> scaledContactPointList = new SideDependentList<>(new RecyclingArrayList<>(FramePoint2D::new),
                                                                                                                      new RecyclingArrayList<>(FramePoint2D::new));

   private final ContactStateProvider contactStateProvider;

   private final PlanarRegionCommand leftRegion = new PlanarRegionCommand();
   private final PlanarRegionCommand rightRegion = new PlanarRegionCommand();
   private final SideDependentList<PlanarRegionCommand> footRegions = new SideDependentList<>(leftRegion, rightRegion);
   private final PlanarRegionCommand combinedRegion = new PlanarRegionCommand();

   private final FramePoint2D tempPoint = new FramePoint2D();

   public BipedalSupportPlanarRegionCalculator(FullHumanoidRobotModel fullRobotModel,
                                               HumanoidReferenceFrames referenceFrames,
                                               RobotContactPointParameters<RobotSide> contactPointParameters,
                                               ContactStateProvider contactStateProviders)
   {
      this.contactStateProvider = contactStateProviders;
      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFullRobotModel(fullRobotModel);
      contactableBodiesFactory.setReferenceFrames(referenceFrames);
      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getControllerFootGroundContactPoints());
      contactableFeet = new SideDependentList<>(contactableBodiesFactory.createFootContactablePlaneBodies());


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
         for (int i = 0; i < contactableFeet.get(robotSide).getContactPoints2d().size(); i++)
         {
            FramePoint2DReadOnly contactPoint = contactableFeet.get(robotSide).getContactPoints2d().get(i);
            scaledContactPointList.get(robotSide).add().setAndScale(scaleFactor, contactPoint);
         }
      }

      if (feetAreInSamePlane(contactStateProvider))
      {
         ReferenceFrame leftSoleFrame = contactableFeet.get(RobotSide.LEFT).getSoleFrame();

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
            if (contactStateProvider.isInContact(robotSide))
            {
               ContactablePlaneBody contactableFoot = contactableFeet.get(robotSide);
               List<FramePoint2D> contactPoints = scaledContactPointList.get(robotSide);
               RigidBodyTransform transformToWorld = contactableFoot.getSoleFrame().getTransformToRoot();

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

   private boolean feetAreInSamePlane(ContactStateProvider contactStateProvider)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (!contactStateProvider.isInContact(robotSide))
         {
            return false;
         }
      }
      ReferenceFrame leftSoleFrame = contactableFeet.get(RobotSide.LEFT).getSoleFrame();
      ReferenceFrame rightSoleFrame = contactableFeet.get(RobotSide.RIGHT).getSoleFrame();
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

   public interface ContactStateProvider
   {
      boolean isInContact(RobotSide robotSide);
   }
}
