package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.util.ArrayList;
import java.util.List;

public class CoMTrajectoryPlannerVisualization
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final boolean VISUALIZE = false;
   private static final double POINT_SIZE = 0.005;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final List<YoFramePoint3D> dcmCornerPoints = new ArrayList<>();
   private final List<YoFramePoint3D> comCornerPoints = new ArrayList<>();
   private final List<YoFramePoint3D> vrpCornerPoints = new ArrayList<>();

   private final CoMTrajectoryPlanner comTrajectoryPlanner;

   private final YoDouble omega;
   private final double gravity;

   private final int maxCapacity;

   public CoMTrajectoryPlannerVisualization(int maxCapacity, CoMTrajectoryPlanner comTrajectoryPlanner, YoDouble omega, double gravity,
                                            YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.maxCapacity = maxCapacity;
      this.comTrajectoryPlanner = comTrajectoryPlanner;
      this.omega = omega;
      this.gravity = gravity;

      for (int i = 0; i < maxCapacity + 1; i++)
      {
         dcmCornerPoints.add(new YoFramePoint3D("dcmCornerPoint" + i, worldFrame, registry));
         comCornerPoints.add(new YoFramePoint3D("comCornerPoint" + i, worldFrame, registry));
         vrpCornerPoints.add(new YoFramePoint3D("vrpCornerPoint" + i, worldFrame, registry));
      }

      String packageName = "dcmPlanner";
      YoGraphicsList graphicsList = new YoGraphicsList(packageName);
      ArtifactList artifactList = new ArtifactList(packageName);

      for (int i = 0; i < dcmCornerPoints.size(); i++)
      {
         YoFramePoint3D dcmCornerPoint = dcmCornerPoints.get(i);
         YoFramePoint3D comCornerPoint = comCornerPoints.get(i);
         YoFramePoint3D vrpCornerPoint = vrpCornerPoints.get(i);
         YoGraphicPosition dcmCornerPointViz = new YoGraphicPosition("DCMCornerPoint" + i, dcmCornerPoint, POINT_SIZE, YoAppearance.Blue(),
                                                                     YoGraphicPosition.GraphicType.BALL);
         YoGraphicPosition comCornerPointViz = new YoGraphicPosition("CoMCornerPoint" + i, comCornerPoint, POINT_SIZE, YoAppearance.Black(),
                                                                     YoGraphicPosition.GraphicType.BALL);
         YoGraphicPosition vrpCornerPointViz = new YoGraphicPosition("VRPCornerPoint" + i, vrpCornerPoint, POINT_SIZE, YoAppearance.Green(),
                                                                     YoGraphicPosition.GraphicType.SOLID_BALL);
         graphicsList.add(dcmCornerPointViz);
         graphicsList.add(comCornerPointViz);
         graphicsList.add(vrpCornerPointViz);

         artifactList.add(dcmCornerPointViz.createArtifact());
         artifactList.add(comCornerPointViz.createArtifact());
         artifactList.add(vrpCornerPointViz.createArtifact());
      }

      artifactList.setVisible(VISUALIZE);
      graphicsList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerYoGraphicsList(graphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);

      parentRegistry.addChild(registry);
   }

   private final FramePoint3D firstCoefficient = new FramePoint3D();
   private final FramePoint3D secondCoefficient = new FramePoint3D();

   private final FrameVector3D comVelocity = new FrameVector3D();
   private final FrameVector3D dcmVelocity = new FrameVector3D();

   public void updateFromTrajectory()
   {
      for (int i = 0; i < maxCapacity; i++)
      {
         ContactState contactState = comTrajectoryPlanner.getContactState(i);
         double firstCoefficientPositionMultiplier = CoMTrajectoryPlannerTools.getFirstCoefficientPositionMultiplier(contactState, 0.0, omega.getDoubleValue());
         double secondCoefficientPositionMultiplier = CoMTrajectoryPlannerTools.getSecondCoefficientPositionMultiplier(contactState, 0.0, omega.getDoubleValue());

         double firstCoefficientVelocityMultiplier = CoMTrajectoryPlannerTools.getFirstCoefficientVelocityMultiplier(contactState, 0.0, omega.getDoubleValue());
         double secondCoefficientVelocityMultiplier = CoMTrajectoryPlannerTools.getSecondCoefficientVelocityMultiplier(contactState, 0.0, omega.getDoubleValue());

         double gravityPositionEffect = CoMTrajectoryPlannerTools.getGravityPositionEffect(contactState, 0.0, gravity);
         double gravityVelocityEffect = CoMTrajectoryPlannerTools.getGravityVelocityEffect(contactState, 0.0, gravity);

         firstCoefficient.setX(comTrajectoryPlanner.getXCoefficientVector().get(CoMTrajectoryPlannerTools.getFirstCoefficientIndex(i)));
         firstCoefficient.setY(comTrajectoryPlanner.getYCoefficientVector().get(CoMTrajectoryPlannerTools.getFirstCoefficientIndex(i)));
         firstCoefficient.setZ(comTrajectoryPlanner.getZCoefficientVector().get(CoMTrajectoryPlannerTools.getFirstCoefficientIndex(i)));

         secondCoefficient.setX(comTrajectoryPlanner.getXCoefficientVector().get(CoMTrajectoryPlannerTools.getSecondCoefficientIndex(i)));
         secondCoefficient.setY(comTrajectoryPlanner.getYCoefficientVector().get(CoMTrajectoryPlannerTools.getSecondCoefficientIndex(i)));
         secondCoefficient.setZ(comTrajectoryPlanner.getZCoefficientVector().get(CoMTrajectoryPlannerTools.getSecondCoefficientIndex(i)));

         FixedFramePoint3DBasics desiredCoMPosition = comCornerPoints.get(i);

         if (contactState == ContactState.IN_CONTACT)
         {
            desiredCoMPosition.set(comTrajectoryPlanner.getCopPosition(i));
            desiredCoMPosition.addZ(comTrajectoryPlanner.getNominalCoMHeight());
         }
         else
         {
            desiredCoMPosition.setToZero();
         }
         desiredCoMPosition.scaleAdd(firstCoefficientPositionMultiplier, firstCoefficient, desiredCoMPosition);
         desiredCoMPosition.scaleAdd(secondCoefficientPositionMultiplier, secondCoefficient, desiredCoMPosition);
         desiredCoMPosition.addZ(gravityPositionEffect);

         comVelocity.setToZero();
         comVelocity.scaleAdd(firstCoefficientVelocityMultiplier, firstCoefficient, comVelocity);
         comVelocity.scaleAdd(secondCoefficientVelocityMultiplier, secondCoefficient, comVelocity);
         comVelocity.addZ(gravityVelocityEffect);

         FixedFramePoint3DBasics desiredDCMPosition = dcmCornerPoints.get(i);
         desiredDCMPosition.scaleAdd(1.0 / omega.getValue(), comVelocity, desiredCoMPosition);

         dcmVelocity.set(firstCoefficient);
         dcmVelocity.scale(2.0 * firstCoefficientVelocityMultiplier);

         vrpCornerPoints.get(i).scaleAdd(-omega.getValue(), dcmVelocity, desiredDCMPosition);
      }
   }

}
