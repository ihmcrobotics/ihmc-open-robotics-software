package us.ihmc.valkyrie.planner;

import controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

public class ValkyrieAStarFootstepPlannerParameters
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final YoDouble idealFootstepWidth = new YoDouble("idealFootstepWidth", registry);
   private final YoDouble minimumFootstepLength = new YoDouble("minimumFootstepLength", registry);
   private final YoDouble idealFootstepLength = new YoDouble("idealFootstepLength", registry);
   private final YoDouble minimumStepWidth = new YoDouble("minimumStepWidth", registry);
   private final YoDouble maximumStepWidth = new YoDouble("maximumStepWidth", registry);
   private final YoDouble maximumStepReach = new YoDouble("maximumStepReach", registry);
   private final YoDouble minimumXClearanceFromStance = new YoDouble("minimumXClearanceFromStance", registry);
   private final YoDouble minimumYClearanceFromStance = new YoDouble("minimumYClearanceFromStance", registry);
   private final YoDouble minimumStepYaw = new YoDouble("minimumStepYaw", registry);
   private final YoDouble maximumStepYaw = new YoDouble("maximumStepYaw", registry);
   private final YoDouble stepYawReductionFactorAtMaxReach = new YoDouble("stepYawReductionFactorAtMaxReach", registry);
   private final YoDouble maximumStepZ = new YoDouble("maximumStepZ", registry);
   private final YoDouble minimumFootholdPercent = new YoDouble("minimumFootholdPercent", registry);
   private final YoDouble maximumSurfanceInclineRadians = new YoDouble("maximumSurfanceInclineRadians", registry);
   private final YoBoolean wiggleWhilePlanning = new YoBoolean("wiggleWhilePlanning", registry);
   private final YoDouble wiggleInsideDelta = new YoDouble("wiggleInsideDelta", registry);
   private final YoDouble maximumXYWiggle = new YoDouble("maximumXYWiggle", registry);
   private final YoDouble maximumYawWiggle = new YoDouble("maximumYawWiggle", registry);
   private final YoDouble cliffHeightToAvoid = new YoDouble("cliffHeightToAvoid", registry);
   private final YoDouble minimumDistanceFromCliffBottoms = new YoDouble("minimumDistanceFromCliffBottoms", registry);
   private final YoDouble flatGroundLowerThreshold = new YoDouble("flatGroundLowerThreshold", registry);
   private final YoDouble flatGroundUpperThreshold = new YoDouble("flatGroundUpperThreshold", registry);
   private final YoDouble maximumStepWidthWhenSteppingDown = new YoDouble("maximumStepWidthWhenSteppingDown", registry);
   private final YoDouble maximumStepReachWhenSteppingDown = new YoDouble("maximumStepReachWhenSteppingDown", registry);
   private final YoDouble maximumStepWidthWhenSteppingUp = new YoDouble("maximumStepWidthWhenSteppingUp", registry);
   private final YoDouble maximumStepReachWhenSteppingUp = new YoDouble("maximumStepReachWhenSteppingUp", registry);
   private final YoDouble translationScaleFromGrandparentNode = new YoDouble("translationScaleFromGrandparentNode", registry);
   private final YoDouble finalTurnProximity = new YoDouble("finalTurnProximity", registry);
   private final YoBoolean checkForPathCollisions = new YoBoolean("checkForPathCollisions", registry);
   private final YoBoolean checkForBodyBoxCollisions = new YoBoolean("checkForBodyBoxCollisions", registry);
   private final YoFrameVector3D bodyBoxDimensions = new YoFrameVector3D("bodyBoxDimensions", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D bodyBoxOffset = new YoFrameVector3D("bodyBoxOffset", ReferenceFrame.getWorldFrame(), registry);
   private final YoInteger numberOfBoundingBoxChecks = new YoInteger("numberOfBoundingBoxChecks", registry);
   private final YoFrameVector3D translationWeight = new YoFrameVector3D("translationWeight", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll orientationWeight = new YoFrameYawPitchRoll("orientationWeight", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble costPerStep = new YoDouble("costPerStep", registry);
   private final YoDouble footholdAreaWeight = new YoDouble("footholdAreaWeight", registry);
   private final YoDouble astarHeuristicsWeight = new YoDouble("astarHeuristicsWeight", registry);

   public ValkyrieAStarFootstepPlannerParameters()
   {
      this(null);
   }

   public ValkyrieAStarFootstepPlannerParameters(YoVariableRegistry parentRegistry)
   {
      setToDefault();
      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public double getIdealFootstepWidth()
   {
      return idealFootstepWidth.getDoubleValue();
   }

   public double getMinimumFootstepLength()
   {
      return minimumFootstepLength.getDoubleValue();
   }

   public double getIdealFootstepLength()
   {
      return idealFootstepLength.getDoubleValue();
   }

   public double getMinimumStepWidth()
   {
      return minimumStepWidth.getDoubleValue();
   }

   public double getMaximumStepWidth()
   {
      return maximumStepWidth.getDoubleValue();
   }

   public double getMaximumStepReach()
   {
      return maximumStepReach.getDoubleValue();
   }

   public double getMinimumXClearanceFromStance()
   {
      return minimumXClearanceFromStance.getDoubleValue();
   }

   public double getMinimumYClearanceFromStance()
   {
      return minimumYClearanceFromStance.getDoubleValue();
   }

   public double getMinimumStepYaw()
   {
      return minimumStepYaw.getDoubleValue();
   }

   public double getMaximumStepYaw()
   {
      return maximumStepYaw.getDoubleValue();
   }

   public double getStepYawReductionFactorAtMaxReach()
   {
      return stepYawReductionFactorAtMaxReach.getDoubleValue();
   }

   public double getMaximumStepZ()
   {
      return maximumStepZ.getDoubleValue();
   }

   public double getMinimumFootholdPercent()
   {
      return minimumFootholdPercent.getDoubleValue();
   }

   public double getMaximumSurfanceInclineRadians()
   {
      return maximumSurfanceInclineRadians.getDoubleValue();
   }

   public boolean getWiggleWhilePlanning()
   {
      return wiggleWhilePlanning.getBooleanValue();
   }

   public double getWiggleInsideDelta()
   {
      return wiggleInsideDelta.getDoubleValue();
   }

   public double getMaximumXYWiggle()
   {
      return maximumXYWiggle.getDoubleValue();
   }

   public double getMaximumYawWiggle()
   {
      return maximumYawWiggle.getDoubleValue();
   }

   public double getCliffHeightToAvoid()
   {
      return cliffHeightToAvoid.getDoubleValue();
   }

   public double getMinimumDistanceFromCliffBottoms()
   {
      return minimumDistanceFromCliffBottoms.getDoubleValue();
   }

   public double getFlatGroundLowerThreshold()
   {
      return flatGroundLowerThreshold.getDoubleValue();
   }

   public double getFlatGroundUpperThreshold()
   {
      return flatGroundUpperThreshold.getDoubleValue();
   }

   public double getMaximumStepWidthWhenSteppingDown()
   {
      return maximumStepWidthWhenSteppingDown.getDoubleValue();
   }

   public double getMaximumStepReachWhenSteppingDown()
   {
      return maximumStepReachWhenSteppingDown.getDoubleValue();
   }

   public double getMaximumStepWidthWhenSteppingUp()
   {
      return maximumStepWidthWhenSteppingUp.getDoubleValue();
   }

   public double getMaximumStepReachWhenSteppingUp()
   {
      return maximumStepReachWhenSteppingUp.getDoubleValue();
   }

   public double getTranslationScaleFromGrandparentNode()
   {
      return translationScaleFromGrandparentNode.getDoubleValue();
   }

   public double getFinalTurnProximity()
   {
      return finalTurnProximity.getDoubleValue();
   }

   public boolean getCheckForPathCollisions()
   {
      return checkForPathCollisions.getBooleanValue();
   }

   public boolean getCheckForBodyBoxCollisions()
   {
      return checkForBodyBoxCollisions.getBooleanValue();
   }

   public Vector3DReadOnly getBodyBoxDimensions()
   {
      return bodyBoxDimensions;
   }

   public Vector3DReadOnly getBodyBoxOffset()
   {
      return bodyBoxOffset;
   }

   public int getNumberOfBoundingBoxChecks()
   {
      return numberOfBoundingBoxChecks.getIntegerValue();
   }

   public Vector3DReadOnly getTranslationWeight()
   {
      return translationWeight;
   }

   public YawPitchRollReadOnly getOrientationWeight()
   {
      return orientationWeight;
   }

   public double getCostPerStep()
   {
      return costPerStep.getDoubleValue();
   }

   public double getFootholdAreaWeight()
   {
      return footholdAreaWeight.getDoubleValue();
   }

   public double getAstarHeuristicsWeight()
   {
      return astarHeuristicsWeight.getDoubleValue();
   }

   public void setIdealFootstepWidth(double idealFootstepWidth)
   {
      this.idealFootstepWidth.set(idealFootstepWidth);
   }

   public void setMinimumFootstepLength(double minimumFootstepLength)
   {
      this.minimumFootstepLength.set(minimumFootstepLength);
   }

   public void setIdealFootstepLength(double idealFootstepLength)
   {
      this.idealFootstepLength.set(idealFootstepLength);
   }

   public void setMinimumStepWidth(double minimumStepWidth)
   {
      this.minimumStepWidth.set(minimumStepWidth);
   }

   public void setMaximumStepWidth(double maximumStepWidth)
   {
      this.maximumStepWidth.set(maximumStepWidth);
   }

   public void setMaximumStepReach(double maximumStepReach)
   {
      this.maximumStepReach.set(maximumStepReach);
   }

   public void setMinimumXClearanceFromStance(double minimumXClearanceFromStance)
   {
      this.minimumXClearanceFromStance.set(minimumXClearanceFromStance);
   }

   public void setMinimumYClearanceFromStance(double minimumYClearanceFromStance)
   {
      this.minimumYClearanceFromStance.set(minimumYClearanceFromStance);
   }

   public void setMinimumStepYaw(double minimumStepYaw)
   {
      this.minimumStepYaw.set(minimumStepYaw);
   }

   public void setMaximumStepYaw(double maximumStepYaw)
   {
      this.maximumStepYaw.set(maximumStepYaw);
   }

   public void setStepYawReductionFactorAtMaxReach(double stepYawReductionFactorAtMaxReach)
   {
      this.stepYawReductionFactorAtMaxReach.set(stepYawReductionFactorAtMaxReach);
   }

   public void setMaximumStepZ(double maximumStepZ)
   {
      this.maximumStepZ.set(maximumStepZ);
   }

   public void setMinimumFootholdPercent(double minimumFootholdPercent)
   {
      this.minimumFootholdPercent.set(minimumFootholdPercent);
   }

   public void setMaximumSurfanceInclineRadians(double maximumSurfanceInclineRadians)
   {
      this.maximumSurfanceInclineRadians.set(maximumSurfanceInclineRadians);
   }

   public void setWiggleInsideDelta(double wiggleInsideDelta)
   {
      this.wiggleInsideDelta.set(wiggleInsideDelta);
   }

   public void setMaximumXYWiggle(double maximumXYWiggle)
   {
      this.maximumXYWiggle.set(maximumXYWiggle);
   }

   public void setMaximumYawWiggle(double maximumYawWiggle)
   {
      this.maximumYawWiggle.set(maximumYawWiggle);
   }

   public void setCliffHeightToAvoid(double cliffHeightToAvoid)
   {
      this.cliffHeightToAvoid.set(cliffHeightToAvoid);
   }

   public void setMinimumDistanceFromCliffBottoms(double minimumDistanceFromCliffBottoms)
   {
      this.minimumDistanceFromCliffBottoms.set(minimumDistanceFromCliffBottoms);
   }

   public void setFlatGroundLowerThreshold(double flatGroundLowerThreshold)
   {
      this.flatGroundLowerThreshold.set(flatGroundLowerThreshold);
   }

   public void setFlatGroundUpperThreshold(double flatGroundUpperThreshold)
   {
      this.flatGroundUpperThreshold.set(flatGroundUpperThreshold);
   }

   public void setMaximumStepWidthWhenSteppingDown(double maximumStepWidthWhenSteppingDown)
   {
      this.maximumStepWidthWhenSteppingDown.set(maximumStepWidthWhenSteppingDown);
   }

   public void setMaximumStepReachWhenSteppingDown(double maximumStepReachWhenSteppingDown)
   {
      this.maximumStepReachWhenSteppingDown.set(maximumStepReachWhenSteppingDown);
   }

   public void setMaximumStepWidthWhenSteppingUp(double maximumStepWidthWhenSteppingUp)
   {
      this.maximumStepWidthWhenSteppingUp.set(maximumStepWidthWhenSteppingUp);
   }

   public void setMaximumStepReachWhenSteppingUp(double maximumStepReachWhenSteppingUp)
   {
      this.maximumStepReachWhenSteppingUp.set(maximumStepReachWhenSteppingUp);
   }

   public void setTranslationScaleFromGrandparentNode(double translationScaleFromGrandparentNode)
   {
      this.translationScaleFromGrandparentNode.set(translationScaleFromGrandparentNode);
   }

   public void setFinalTurnProximity(double finalTurnProximity)
   {
      this.finalTurnProximity.set(finalTurnProximity);
   }

   public void setCheckForPathCollisions(boolean checkForPathCollisions)
   {
      this.checkForPathCollisions.set(checkForPathCollisions);
   }

   public void setCheckForBodyBoxCollisions(boolean checkForBodyBoxCollisions)
   {
      this.checkForBodyBoxCollisions.set(checkForBodyBoxCollisions);
   }

   public void setBodyBoxDimensionX(double bodyBoxDimensionX)
   {
      this.bodyBoxDimensions.setX(bodyBoxDimensionX);
   }

   public void setBodyBoxDimensionY(double bodyBoxDimensionY)
   {
      this.bodyBoxDimensions.setY(bodyBoxDimensionY);
   }

   public void setBodyBoxDimensionZ(double bodyBoxDimensionY)
   {
      this.bodyBoxDimensions.setY(bodyBoxDimensionY);
   }

   public void setBodyBoxOffsetX(double bodyBoxOffsetX)
   {
      this.bodyBoxOffset.setX(bodyBoxOffsetX);
   }

   public void setBodyBoxOffsetY(double bodyBoxOffsetY)
   {
      this.bodyBoxOffset.setY(bodyBoxOffsetY);
   }

   public void setBodyBoxOffsetZ(double bodyBoxOffsetZ)
   {
      this.bodyBoxOffset.setZ(bodyBoxOffsetZ);
   }

   public void setNumberOfBoundingBoxChecks(int numberOfBoundingBoxChecks)
   {
      this.numberOfBoundingBoxChecks.set(numberOfBoundingBoxChecks);
   }

   public void setTranslationWeightX(double translationWeightX)
   {
      this.translationWeight.setX(translationWeightX);
   }

   public void setTranslationWeightY(double translationWeightY)
   {
      this.translationWeight.setY(translationWeightY);
   }

   public void setTranslationWeightZ(double translationWeightZ)
   {
      this.translationWeight.setZ(translationWeightZ);
   }

   public void setOrientationWeightYaw(double orientationWeightYaw)
   {
      this.orientationWeight.setYaw(orientationWeightYaw);
   }

   public void setOrientationWeightPitch(double orientationWeightPitch)
   {
      this.orientationWeight.setPitch(orientationWeightPitch);
   }

   public void setOrientationWeightRoll(double orientationWeightRoll)
   {
      this.orientationWeight.setRoll(orientationWeightRoll);
   }

   public void setCostPerStep(double costPerStep)
   {
      this.costPerStep.set(costPerStep);
   }

   public void setFootholdAreaWeight(double footholdAreaWeight)
   {
      this.footholdAreaWeight.set(footholdAreaWeight);
   }

   public void setAstarHeuristicsWeight(double astarHeuristicsWeight)
   {
      this.astarHeuristicsWeight.set(astarHeuristicsWeight);
   }

   public void setFromPacket(ValkyrieFootstepPlannerParametersPacket packet)
   {
      idealFootstepWidth.set(packet.getIdealFootstepWidth());
      minimumFootstepLength.set(packet.getMinimumStepLength());
      idealFootstepLength.set(packet.getIdealFootstepLength());
      minimumStepWidth.set(packet.getMinimumStepWidth());
      maximumStepWidth.set(packet.getMaximumStepWidth());
      maximumStepReach.set(packet.getMaximumStepReach());
      minimumXClearanceFromStance.set(packet.getMinXClearanceFromStance());
      minimumYClearanceFromStance.set(packet.getMinYClearanceFromStance());
      minimumStepYaw.set(packet.getMinimumStepYaw());
      maximumStepYaw.set(packet.getMaximumStepYaw());
      stepYawReductionFactorAtMaxReach.set(packet.getStepYawReductionFactorAtMaxReach());
      maximumStepZ.set(packet.getMaximumStepZ());
      minimumFootholdPercent.set(packet.getMinimumFootholdPercent());
      maximumSurfanceInclineRadians.set(packet.getMaximumSurfaceInclineRadians());
      wiggleWhilePlanning.set(packet.getWiggleWhilePlanning());
      wiggleInsideDelta.set(packet.getWiggleInsideDelta());
      maximumXYWiggle.set(packet.getMaximumXyWiggleDistance());
      maximumYawWiggle.set(packet.getMaximumYawWiggle());
      cliffHeightToAvoid.set(packet.getCliffHeightToAvoid());
      minimumDistanceFromCliffBottoms.set(packet.getMinimumDistanceFromCliffBottoms());
      flatGroundLowerThreshold.set(packet.getFlatGroundLowerThreshold());
      flatGroundUpperThreshold.set(packet.getFlatGroundUpperThreshold());
      maximumStepWidthWhenSteppingDown.set(packet.getMaximumStepWidthWhenSteppingDown());
      maximumStepReachWhenSteppingDown.set(packet.getMaximumStepReachWhenSteppingDown());
      maximumStepWidthWhenSteppingUp.set(packet.getMaximumStepWidthWhenSteppingUp());
      maximumStepReachWhenSteppingUp.set(packet.getMaximumStepReachWhenSteppingUp());
      translationScaleFromGrandparentNode.set(packet.getTranslationScaleFromGrandparentNode());
      finalTurnProximity.set(packet.getFinalTurnProximity());
      checkForPathCollisions.set(packet.getCheckForPathCollisions());
      checkForBodyBoxCollisions.set(packet.getCheckForBodyBoxCollisions());
      bodyBoxDimensions.set(packet.getBodyBoxDimensions());
      bodyBoxOffset.set(packet.getBodyBoxOffset());
      numberOfBoundingBoxChecks.set((int) packet.getNumberOfBoundingBoxChecks());
      translationWeight.set(packet.getTranslationWeight());
      orientationWeight.setEuler(packet.getOrientationWeight());
      costPerStep.set(packet.getCostPerStep());
      footholdAreaWeight.set(packet.getFootholdAreaWeight());
      astarHeuristicsWeight.set(packet.getAStarHeuristicsWeight());
   }

   public void setPacket(ValkyrieFootstepPlannerParametersPacket packet)
   {
      packet.setIdealFootstepWidth(idealFootstepWidth.getDoubleValue());
      packet.setMinimumStepLength(minimumFootstepLength.getDoubleValue());
      packet.setIdealFootstepLength(idealFootstepLength.getDoubleValue());
      packet.setMinimumStepWidth(minimumStepWidth.getDoubleValue());
      packet.setMaximumStepWidth(maximumStepWidth.getDoubleValue());
      packet.setMaximumStepReach(maximumStepReach.getDoubleValue());
      packet.setMinXClearanceFromStance(minimumXClearanceFromStance.getDoubleValue());
      packet.setMinYClearanceFromStance(minimumYClearanceFromStance.getDoubleValue());
      packet.setMinimumStepYaw(minimumStepYaw.getDoubleValue());
      packet.setMaximumStepYaw(maximumStepYaw.getDoubleValue());
      packet.setStepYawReductionFactorAtMaxReach(stepYawReductionFactorAtMaxReach.getDoubleValue());
      packet.setMaximumStepZ(maximumStepZ.getDoubleValue());
      packet.setMinimumFootholdPercent(minimumFootholdPercent.getDoubleValue());
      packet.setMaximumSurfaceInclineRadians(maximumSurfanceInclineRadians.getDoubleValue());
      packet.setWiggleWhilePlanning(wiggleWhilePlanning.getBooleanValue());
      packet.setWiggleInsideDelta(wiggleInsideDelta.getDoubleValue());
      packet.setMaximumXyWiggleDistance(maximumXYWiggle.getDoubleValue());
      packet.setMaximumYawWiggle(maximumYawWiggle.getDoubleValue());
      packet.setCliffHeightToAvoid(cliffHeightToAvoid.getDoubleValue());
      packet.setMinimumDistanceFromCliffBottoms(minimumDistanceFromCliffBottoms.getDoubleValue());
      packet.setFlatGroundLowerThreshold(flatGroundLowerThreshold.getDoubleValue());
      packet.setFlatGroundUpperThreshold(flatGroundUpperThreshold.getDoubleValue());
      packet.setMaximumStepWidthWhenSteppingDown(maximumStepWidthWhenSteppingDown.getDoubleValue());
      packet.setMaximumStepReachWhenSteppingDown(maximumStepReachWhenSteppingDown.getDoubleValue());
      packet.setMaximumStepWidthWhenSteppingUp(maximumStepWidthWhenSteppingUp.getDoubleValue());
      packet.setMaximumStepReachWhenSteppingUp(maximumStepReachWhenSteppingUp.getDoubleValue());
      packet.setTranslationScaleFromGrandparentNode(translationScaleFromGrandparentNode.getDoubleValue());
      packet.setFinalTurnProximity(finalTurnProximity.getDoubleValue());
      packet.setCheckForPathCollisions(checkForPathCollisions.getBooleanValue());
      packet.setCheckForBodyBoxCollisions(checkForBodyBoxCollisions.getBooleanValue());
      packet.getBodyBoxDimensions().set(bodyBoxDimensions);
      packet.getBodyBoxOffset().set(bodyBoxOffset);
      packet.setNumberOfBoundingBoxChecks(numberOfBoundingBoxChecks.getIntegerValue());
      packet.getTranslationWeight().set(translationWeight);
      packet.getOrientationWeight().set(orientationWeight.getYawPitchRoll());
      packet.setCostPerStep(costPerStep.getDoubleValue());
      packet.setFootholdAreaWeight(footholdAreaWeight.getDoubleValue());
      packet.setAStarHeuristicsWeight(astarHeuristicsWeight.getDoubleValue());
   }

   public void setToDefault()
   {
      idealFootstepWidth.set(0.2);
      minimumFootstepLength.set(-0.4);
      idealFootstepLength.set(0.2);
      minimumStepWidth.set(0.18);
      maximumStepWidth.set(0.4);
      maximumStepReach.set(0.5);
      minimumXClearanceFromStance.set(0.2);
      minimumYClearanceFromStance.set(0.2);
      minimumStepYaw.set(-0.15);
      maximumStepYaw.set(0.6);
      stepYawReductionFactorAtMaxReach.set(0.5);
      maximumStepZ.set(0.15);
      minimumFootholdPercent.set(0.5);
      maximumSurfanceInclineRadians.set(Math.toRadians(45.0));
      wiggleWhilePlanning.set(false);
      wiggleInsideDelta.set(0.03);
      maximumXYWiggle.set(0.04);
      maximumYawWiggle.set(0.3);
      cliffHeightToAvoid.set(0.07);
      minimumDistanceFromCliffBottoms.set(0.04);
      flatGroundLowerThreshold.set(0.05);
      flatGroundUpperThreshold.set(0.1);
      maximumStepWidthWhenSteppingDown.set(0.3);
      maximumStepReachWhenSteppingDown.set(0.4);
      maximumStepWidthWhenSteppingUp.set(0.25);
      maximumStepReachWhenSteppingUp.set(0.4);
      translationScaleFromGrandparentNode.set(1.5);
      finalTurnProximity.set(0.75);
      checkForPathCollisions.set(true);
      checkForBodyBoxCollisions.set(false);
      bodyBoxDimensions.set(0.4, 0.85, 1.5);
      bodyBoxOffset.set(0.03, 0.2, 0.1);
      numberOfBoundingBoxChecks.set(1);
      translationWeight.set(2.5, 1.0, 0.0);
      orientationWeight.set(0.15, 0.0, 0.0);
      costPerStep.set(0.5);
      footholdAreaWeight.set(4.0);
      astarHeuristicsWeight.set(5.0);
   }
}
