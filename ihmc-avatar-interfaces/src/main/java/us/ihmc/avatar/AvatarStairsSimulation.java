package us.ihmc.avatar;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessorParameters;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotEnvironmentAwareness.tools.ConstantPlanarRegionsPublisher;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;

public class AvatarStairsSimulation
{
   private final RequiredFactoryField<DRCRobotModel> robotModel = new RequiredFactoryField<>("robotModel");
   private final OptionalFactoryField<Pose3D> startingPose = new OptionalFactoryField<>("startingPose");
   private final OptionalFactoryField<Integer> numberOfSteps = new OptionalFactoryField<>("numberOfSteps");
   private final OptionalFactoryField<Double> stepDepth = new OptionalFactoryField<>("stepDepth");
   private final OptionalFactoryField<Double> stepHeight = new OptionalFactoryField<>("stepHeight");
   private final OptionalFactoryField<Double> stepWidth = new OptionalFactoryField<>("stepWidth");
   private final OptionalFactoryField<Double> bottomPlatformWidth = new OptionalFactoryField<>("bottomPlatformWidth");
   private final OptionalFactoryField<Double> bottomPlatformLength = new OptionalFactoryField<>("bottomPlatformLength");
   private final OptionalFactoryField<Double> topPlatformWidth = new OptionalFactoryField<>("topPlatformWidth");
   private final OptionalFactoryField<Double> topPlatformLength = new OptionalFactoryField<>("topPlatformLength");
   private final OptionalFactoryField<Boolean> placeRobotAtTop = new OptionalFactoryField<>("placeRobotAtTop");

   public void setRobotModel(DRCRobotModel robotModel)
   {
      this.robotModel.set(robotModel);
   }

   public void setStartingPose(Pose3D startingPose)
   {
      this.startingPose.set(startingPose);
   }

   public void setNumberOfSteps(int numberOfSteps)
   {
      this.numberOfSteps.set(numberOfSteps);
   }

   public void setStepDepth(double stepDepth)
   {
      this.stepDepth.set(stepDepth);
   }

   public void setStepHeight(double stepHeight)
   {
      this.stepHeight.set(stepHeight);
   }

   public void setStepWidth(double stepWidth)
   {
      this.stepWidth.set(stepWidth);
   }

   public void setBottomPlatformWidth(double bottomPlatformWidth)
   {
      this.bottomPlatformWidth.set(bottomPlatformWidth);
   }

   public void setBottomPlatformLength(double bottomPlatformLength)
   {
      this.bottomPlatformLength.set(bottomPlatformLength);
   }

   public void setTopPlatformWidth(double topPlatformWidth)
   {
      this.topPlatformWidth.set(topPlatformWidth);
   }

   public void setTopPlatformLength(double topPlatformLength)
   {
      this.topPlatformLength.set(topPlatformLength);
   }

   public void setPlaceRobotAtTop(boolean placeRobotAtTop)
   {
      this.placeRobotAtTop.set(placeRobotAtTop);
   }

   public void startSimulation()
   {
      startingPose.setDefaultValue(new Pose3D());
      numberOfSteps.setDefaultValue(5);
      stepDepth.setDefaultValue(toMeters(11.0));
      stepHeight.setDefaultValue(toMeters(6.75));
      stepWidth.setDefaultValue(1.0);
      bottomPlatformWidth.setDefaultValue(2.0);
      bottomPlatformLength.setDefaultValue(2.0);
      topPlatformWidth.setDefaultValue(1.0);
      topPlatformLength.setDefaultValue(1.0);

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      RigidBodyTransform initialTransform = new RigidBodyTransform();
      startingPose.get().get(initialTransform);
      generator.setTransform(initialTransform);

      // bottom platform
      generator.addRectangle(bottomPlatformLength.get(), bottomPlatformWidth.get());

      // stairs
      generator.translate(0.5 * (bottomPlatformLength.get() - stepDepth.get()), 0.0, 0.0);
      for (int i = 0; i < numberOfSteps.get() - 1; i++)
      {
         generator.translate(stepDepth.get(), 0.0, stepHeight.get());
         generator.addRectangle(stepDepth.get(), stepWidth.get());
      }

      generator.translate(0.5 * (topPlatformLength.get() + stepDepth.get()), 0.0, stepHeight.get());
      generator.addRectangle(topPlatformLength.get(), topPlatformWidth.get());
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      PlanarRegionsListDefinedEnvironment simEnvironment = new PlanarRegionsListDefinedEnvironment(planarRegionsList, 0.02, true);
      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel.get(), simEnvironment);
      simulationStarter.setRunMultiThreaded(true);
      simulationStarter.setInitializeEstimatorToActual(true);

      if (placeRobotAtTop.get())
      {
         Pose3D robotPose = new Pose3D(startingPose.get());
         robotPose.appendTranslation(0.5 * bottomPlatformLength.get(), 0.0, 0.0);
         robotPose.appendTranslation(numberOfSteps.get() * stepDepth.get(), 0.0, numberOfSteps.get() * stepHeight.get());
         robotPose.appendYawRotation(Math.PI);

         simulationStarter.setStartingLocation(() -> new OffsetAndYawRobotInitialSetup(robotPose.getPosition(), robotPose.getYaw()));
      }
      else
      {
         simulationStarter.setStartingLocation(() -> new OffsetAndYawRobotInitialSetup(startingPose.get().getPosition(), startingPose.get().getYaw()));
      }

      HumanoidNetworkProcessorParameters networkProcessorParameters = new HumanoidNetworkProcessorParameters();

      // talk to controller and footstep planner
      networkProcessorParameters.setUseFootstepPlanningToolboxModule(false);
      networkProcessorParameters.setUseWalkingPreviewModule(true);
      networkProcessorParameters.setUseBipedalSupportPlanarRegionPublisherModule(true);

      // disable everything else
      networkProcessorParameters.setUseSensorModule(true);
      networkProcessorParameters.setUseHumanoidAvatarREAStateUpdater(true);

      // start sim
      simulationStarter.startSimulation(networkProcessorParameters, false);

      // spoof and publish planar regions
      ConstantPlanarRegionsPublisher constantPlanarRegionsPublisher = new ConstantPlanarRegionsPublisher(planarRegionsList);
      constantPlanarRegionsPublisher.start(2000);
   }

   private static double toMeters(double inches)
   {
      double inchesPerMeter = 39.3701;
      return inches / inchesPerMeter;
   }
}
