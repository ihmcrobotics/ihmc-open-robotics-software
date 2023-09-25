package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.math.filters.GlitchFilteredYoInteger;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class FootContactStateEstimator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double minForceZInPercentThresholdToFilterFoot = 0.0;
   private static final double maxForceZInPercentThresholdToFilterFoot = 0.45;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final List<RigidBodyBasics> feet = new ArrayList<RigidBodyBasics>();

   private final YoInteger numberOfEndEffectorsTrusted = new YoInteger("numberOfEndEffectorsTrusted", registry);
   private final YoInteger numberOfEndEffectorsFilteredByLoad = new YoInteger("numberOfEndEffectorsFilteredByLoad", registry);

   private final BooleanProvider trustImuWhenNoFeetAreInContact;
   private final Map<RigidBodyBasics, YoDouble> footForcesZInPercentOfTotalForce = new LinkedHashMap<>();
   private final IntegerProvider optimalNumberOfTrustedFeet;
   private final DoubleProvider forceZInPercentThresholdToTrustFoot;
   private final DoubleProvider forceZInPercentThresholdToNotTrustFoot;

   private final Map<RigidBodyBasics, FootSwitchInterface> footSwitches;
   private final Map<RigidBodyBasics, FixedFrameVector3DBasics> footForces = new LinkedHashMap<>();
   private final Map<RigidBodyBasics, Wrench> footWrenches = new LinkedHashMap<>();
   private final DoubleProvider delayTimeBeforeTrustingFoot;
   private final Map<RigidBodyBasics, GlitchFilteredYoBoolean> haveFeetHitGroundFiltered = new LinkedHashMap<>();
   private final Map<RigidBodyBasics, YoBoolean> areFeetTrusted = new LinkedHashMap<>();
   private final Map<RigidBodyBasics, YoBoolean> wereFeetTrustedLastTick = new LinkedHashMap<>();
   private final List<RigidBodyBasics> listOfTrustedFeet = new ArrayList<>();
   private final List<RigidBodyBasics> listOfUnTrustedFeet = new ArrayList<>();

   private enum SlippageCompensatorMode
   {
      LOAD_THRESHOLD, MIN_PELVIS_ACCEL
   };

   private final YoEnum<SlippageCompensatorMode> slippageCompensatorMode = new YoEnum<SlippageCompensatorMode>("slippageCompensatorMode",
                                                                                                               registry,
                                                                                                               SlippageCompensatorMode.class);

   private final BooleanProvider trustOnlyLowestFoot = new BooleanParameter("TrustOnlyLowestFoot", registry, false);
   private final IntegerProvider lowestFootWindowSize = new IntegerParameter("LowestFootWindowSize", registry, 0);
   private final GlitchFilteredYoInteger lowestFootInContactIndex = new GlitchFilteredYoInteger("LowestFootInContact", lowestFootWindowSize, registry);

   private final double estimatorDT;

   public FootContactStateEstimator(Map<RigidBodyBasics, FootSwitchInterface> footSwitches,
                                    StateEstimatorParameters stateEstimatorParameters,
                                    YoRegistry parentRegistry)
   {
      this.footSwitches = footSwitches;
      this.estimatorDT = stateEstimatorParameters.getEstimatorDT();
      this.feet.addAll(footSwitches.keySet());

      delayTimeBeforeTrustingFoot = new DoubleParameter("delayTimeBeforeTrustingFoot", registry, stateEstimatorParameters.getDelayTimeForTrustingFoot());
      optimalNumberOfTrustedFeet = new IntegerParameter("optimalNumberOfTrustedFeet", registry, 2);
      forceZInPercentThresholdToTrustFoot = new DoubleParameter("forceZInPercentThresholdToTrustFoot",
                                                                registry,
                                                                stateEstimatorParameters.getForceInPercentOfWeightThresholdToTrustFoot());
      forceZInPercentThresholdToNotTrustFoot = new DoubleParameter("forceZInPercentThresholdToNotTrustFoot",
                                                                   registry,
                                                                   stateEstimatorParameters.getForceInPercentOfWeightThresholdToNotTrustFoot());
      trustImuWhenNoFeetAreInContact = new BooleanParameter("trustImuWhenNoFeetAreInContact",
                                                            registry,
                                                            stateEstimatorParameters.getPelvisLinearStateUpdaterTrustImuWhenNoFeetAreInContact());

      slippageCompensatorMode.set(SlippageCompensatorMode.LOAD_THRESHOLD);

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBodyBasics foot = feet.get(i);
         String footPrefix = foot.getName();

         final GlitchFilteredYoBoolean hasFootHitTheGroundFiltered = new GlitchFilteredYoBoolean("has" + footPrefix + "FootHitGroundFiltered", registry, 0);
         hasFootHitTheGroundFiltered.set(true);
         haveFeetHitGroundFiltered.put(foot, hasFootHitTheGroundFiltered);

         YoBoolean isFootTrusted = new YoBoolean("is" + footPrefix + "FootTrusted", registry);
         YoBoolean wasFootTrusted = new YoBoolean("was" + footPrefix + "FootTrustedLastTick", registry);
         if (i == 0)
         {
            isFootTrusted.set(true);
            wasFootTrusted.set(true);
         }
         areFeetTrusted.put(foot, isFootTrusted);
         wereFeetTrustedLastTick.put(foot, wasFootTrusted);

         YoDouble footForceZInPercentOfTotalForce = new YoDouble(footPrefix + "FootForceZInPercentOfTotalForce", registry);
         footForcesZInPercentOfTotalForce.put(foot, footForceZInPercentOfTotalForce);

         footForces.put(foot, new FrameVector3D(worldFrame));
         footWrenches.put(foot, new Wrench());
      }

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      numberOfEndEffectorsTrusted.set(setTrustedFeetUsingFootSwitches());
      numberOfEndEffectorsFilteredByLoad.set(0);

      if (numberOfEndEffectorsTrusted.getIntegerValue() >= optimalNumberOfTrustedFeet.getValue())
      {
         switch (slippageCompensatorMode.getEnumValue())
         {
            case LOAD_THRESHOLD:
               int filteredNumberOfEndEffectorsTrusted = filterTrustedFeetBasedOnContactForces(numberOfEndEffectorsTrusted.getIntegerValue());
               numberOfEndEffectorsTrusted.set(filteredNumberOfEndEffectorsTrusted);
               break;
            case MIN_PELVIS_ACCEL:
               throw new RuntimeException("Implement me if possible!");
            default:
               throw new RuntimeException("Should not get there");
         }
      }

      updateTrustedFeetLists();
   }

   private void updateTrustedFeetLists()
   {
      listOfTrustedFeet.clear();
      listOfUnTrustedFeet.clear();

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBodyBasics foot = feet.get(i);
         if (areFeetTrusted.get(foot).getValue())
            listOfTrustedFeet.add(foot);
         else
            listOfUnTrustedFeet.add(foot);
      }
   }

   private int setTrustedFeetUsingFootSwitches()
   {
      int numberOfEndEffectorsTrusted = 0;

      int windowSize = (int) (delayTimeBeforeTrustingFoot.getValue() / estimatorDT);

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBodyBasics foot = feet.get(i);
         wereFeetTrustedLastTick.get(foot).set(areFeetTrusted.get(foot).getValue());
         haveFeetHitGroundFiltered.get(foot).setWindowSize(windowSize);

         if (footSwitches.get(foot).hasFootHitGroundFiltered())
            haveFeetHitGroundFiltered.get(foot).update(true);
         else
            haveFeetHitGroundFiltered.get(foot).set(false);

         if (haveFeetHitGroundFiltered.get(foot).getValue())
            numberOfEndEffectorsTrusted++;
      }

      // Update only if at least one foot hit the ground
      if (numberOfEndEffectorsTrusted > 0)
      {
         if (trustOnlyLowestFoot.getValue())
         {
            numberOfEndEffectorsTrusted = filterAndTrustLowestFoot();
         }
         else
         {
            for (int i = 0; i < feet.size(); i++)
            {
               RigidBodyBasics foot = feet.get(i);
               boolean isFootOnGround = haveFeetHitGroundFiltered.get(foot).getValue();
               areFeetTrusted.get(foot).set(isFootOnGround);
            }
         }
      }
      // Else if there is a foot with a force past the threshold trust the force and not the CoP
      else
      {
         RigidBodyBasics trustedFoot = null;

         for (int i = 0; i < feet.size(); i++)
         {
            RigidBodyBasics foot = feet.get(i);
            if (footSwitches.get(foot).hasFootHitGroundSensitive())
            {
               trustedFoot = foot;
               numberOfEndEffectorsTrusted = 1;
               break;
            }
         }

         if (trustedFoot != null)
         {
            for (int i = 0; i < feet.size(); i++)
            {
               RigidBodyBasics foot = feet.get(i);
               areFeetTrusted.get(foot).set(foot == trustedFoot);
            }
         }
      }

      if (numberOfEndEffectorsTrusted == 0)
      {
         if (trustImuWhenNoFeetAreInContact.getValue())
         {
            for (int i = 0; i < feet.size(); i++)
            {
               RigidBodyBasics foot = feet.get(i);
               areFeetTrusted.get(foot).set(false);
            }
         }
         else
         {
            for (int i = 0; i < feet.size(); i++)
            {
               RigidBodyBasics foot = feet.get(i);
               if (areFeetTrusted.get(foot).getValue())
                  numberOfEndEffectorsTrusted++;
            }
         }
      }

      return numberOfEndEffectorsTrusted;
   }

   private int filterAndTrustLowestFoot()
   {
      int lastLowestFootIdx = lowestFootInContactIndex.getValue();
      int lowestFootIdx = findLowestFootInContact();

      if (haveFeetHitGroundFiltered.get(feet.get(lastLowestFootIdx)).getValue())
      {
         // If the previously trusted foot is still in contact glitch filter the trusted foot to avoid
         // jumping between the feet.
         lowestFootInContactIndex.update(lowestFootIdx);
      }
      else
      {
         // In case the previously trusted foot is not in contact anymore we do not need to glitch filter.
         // Just use the new lowest foot.
         lowestFootInContactIndex.set(lowestFootIdx);
      }

      for (int footIdx = 0; footIdx < feet.size(); footIdx++)
      {
         areFeetTrusted.get(feet.get(footIdx)).set(footIdx == lowestFootInContactIndex.getValue());
      }

      return 1;
   }

   FramePoint3D tmpFramePoint = new FramePoint3D();

   private int findLowestFootInContact()
   {
      int lowestFootInContact = -1;
      double lowestFootZ = Double.MAX_VALUE;
      for (int footIdx = 0; footIdx < feet.size(); footIdx++)
      {
         RigidBodyBasics foot = feet.get(footIdx);
         tmpFramePoint.setToZero(foot.getBodyFixedFrame());
         tmpFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
         double footZ = tmpFramePoint.getZ();

         if (haveFeetHitGroundFiltered.get(foot).getValue())
         {
            if (footZ < lowestFootZ)
            {
               lowestFootZ = footZ;
               lowestFootInContact = footIdx;
            }
         }
      }
      return lowestFootInContact;
   }

   private int filterTrustedFeetBasedOnContactForces(int numberOfEndEffectorsTrusted)
   {
      double totalForceZ = 0.0;
      for (int i = 0; i < feet.size(); i++)
      {
         RigidBodyBasics foot = feet.get(i);
         if (!areFeetTrusted.get(foot).getValue())
            continue;
         Wrench footWrench = footWrenches.get(foot);
         footSwitches.get(foot).getMeasuredWrench(footWrench);
         FixedFrameVector3DBasics footForce = footForces.get(foot);
         footForce.setMatchingFrame(footWrench.getLinearPart());
         totalForceZ += footForce.getZ();
      }

      int filteredNumberOfEndEffectorsTrusted = 0;

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBodyBasics foot = feet.get(i);
         if (!areFeetTrusted.get(foot).getValue())
            continue;

         FixedFrameVector3DBasics footForce = footForces.get(foot);
         YoDouble footLoad = footForcesZInPercentOfTotalForce.get(foot);
         footLoad.set(footForce.getZ() / totalForceZ);

         double percentForceToTrustFootAgain = forceZInPercentThresholdToTrustFoot.getValue();
         double percentForceToNotTrustFoot = forceZInPercentThresholdToNotTrustFoot.getValue();
         percentForceToTrustFootAgain = MathTools.clamp(percentForceToTrustFootAgain,
                                                        minForceZInPercentThresholdToFilterFoot,
                                                        maxForceZInPercentThresholdToFilterFoot);
         percentForceToNotTrustFoot = MathTools.clamp(percentForceToNotTrustFoot,
                                                      minForceZInPercentThresholdToFilterFoot,
                                                      maxForceZInPercentThresholdToFilterFoot);

         double magnitudeForTrust;
         if (wereFeetTrustedLastTick.get(foot).getValue())
            magnitudeForTrust = percentForceToNotTrustFoot;
         else
            magnitudeForTrust = percentForceToTrustFootAgain;

         if (footLoad.getValue() < magnitudeForTrust)
            areFeetTrusted.get(foot).set(false);
         else
            filteredNumberOfEndEffectorsTrusted++;
      }

      numberOfEndEffectorsFilteredByLoad.set(numberOfEndEffectorsTrusted - filteredNumberOfEndEffectorsTrusted);

      return filteredNumberOfEndEffectorsTrusted;
   }

   public int getNumberOfEndEffectorsTrusted()
   {
      return numberOfEndEffectorsTrusted.getValue();
   }

   public List<RigidBodyBasics> getListOfTrustedFeet()
   {
      return listOfTrustedFeet;
   }

   public List<RigidBodyBasics> getListOfUnTrustedFeet()
   {
      return listOfUnTrustedFeet;
   }
}
