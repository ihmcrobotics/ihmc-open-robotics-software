package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.controlModules.YoSE3OffsetFrame;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
import us.ihmc.robotics.dataStructures.YoMutableFrameSpatialVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoMutableFrameSpatialVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoMutableFrameVector3D;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.RateLimitedYoMutableFrameVector3D;
import us.ihmc.robotics.math.filters.RateLimitedYoMutableSpatialVector;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFramePoint3D;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFramePose3D;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFrameQuaternion;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFrameVector3D;

/**
 * {@code FeedbackControllerToolbox} is meant to be used only in the
 * {@link WholeBodyFeedbackController}.
 * <p>
 * It is used as a factory for creating a unique set of {@code YoVariable}s used by the feedback
 * controllers. For instance, when a {@code YoFramePoint} has already been created by a controller,
 * the same object will be given to be next controller needing it.
 * </p>
 *
 * @author Sylvain Bertrand
 *
 */
public class FeedbackControllerToolbox implements FeedbackControllerDataReadOnly
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   public static final String centerOfMassName = "centerOfMass";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final List<Pair<? extends Clearable, List<YoBoolean>>> clearableData = new ArrayList<>();

   private final Map<RigidBodyBasics, EnumMap<Type, Pair<YoMutableFramePoint3D, List<YoBoolean>>>> endEffectorPositions = new HashMap<>();
   private final Map<RigidBodyBasics, EnumMap<Type, Pair<YoMutableFrameQuaternion, List<YoBoolean>>>> endEffectorOrientations = new HashMap<>();
   private final Map<RigidBodyBasics, EnumMap<Type, EnumMap<Space, Pair<YoMutableFrameVector3D, List<YoBoolean>>>>> endEffectorDataVectors = new HashMap<>();
   private final Map<RigidBodyBasics, EnumMap<Space, Pair<RateLimitedYoMutableFrameVector3D, List<YoBoolean>>>> endEffectorRateLimitedDataVectors = new HashMap<>();
   private final Map<RigidBodyBasics, EnumMap<Space, Pair<AlphaFilteredYoMutableFrameVector3D, List<YoBoolean>>>> endEffectorFilteredDataVectors = new HashMap<>();

   private final Map<RigidBodyBasics, YoPID3DGains> endEffectorOrientationGains = new HashMap<>();
   private final Map<RigidBodyBasics, YoPID3DGains> endEffectorPositionGains = new HashMap<>();

   private final Map<RigidBodyBasics, YoSE3OffsetFrame> endEffectorControlFrames = new HashMap<>();

   private final EnumMap<Type, Pair<YoMutableFramePoint3D, List<YoBoolean>>> centerOfMassPositions = new EnumMap<>(Type.class);
   private final EnumMap<Type, EnumMap<Space, Pair<YoMutableFrameVector3D, List<YoBoolean>>>> centerOfMassDataVectors = new EnumMap<>(Type.class);
   private final EnumMap<Space, Pair<RateLimitedYoMutableFrameVector3D, List<YoBoolean>>> centerOfMassRateLimitedDataVectors = new EnumMap<>(Space.class);
   private final EnumMap<Space, Pair<AlphaFilteredYoMutableFrameVector3D, List<YoBoolean>>> centerOfMassFilteredDataVectors = new EnumMap<>(Space.class);
   private YoPID3DGains centerOfMassPositionGains;

   private final Map<String, DoubleProvider> errorVelocityFilterBreakFrequencies;

   public FeedbackControllerToolbox(YoVariableRegistry parentRegistry)
   {
      this(FeedbackControllerSettings.getDefault(), parentRegistry);
   }

   public FeedbackControllerToolbox(FeedbackControllerSettings settings, YoVariableRegistry parentRegistry)
   {
      errorVelocityFilterBreakFrequencies = new HashMap<>();

      List<GroupParameter<Double>> parameters = settings.getErrorVelocityFilterBreakFrequencies();
      if (parameters != null)
      {
         for (GroupParameter<Double> groupParameter : parameters)
         {
            String parameterName = groupParameter.getGroupName() + "ErrorVelocityBreakFrequency";
            DoubleParameter groupBreakFrequency = new DoubleParameter(parameterName, registry, groupParameter.getParameter());
            groupParameter.getMemberNames().forEach(name -> errorVelocityFilterBreakFrequencies.put(name, groupBreakFrequency));
         }
      }

      parentRegistry.addChild(registry);
   }

   /**
    * Retrieves and returns the {@code YoMutableFramePoint3D} for the center of mass associated with the given
    * {@code type}, if it does not exist it is created.
    *
    * @param type the type of the data to retrieve.
    * @return the unique {@code YoMutableFramePoint3D} matching the search criterion.
    */
   public YoMutableFramePoint3D getCenterOfMassPosition(Type type, YoBoolean enabled)
   {
      Pair<YoMutableFramePoint3D, List<YoBoolean>> yoFramePointEnabledPair = centerOfMassPositions.get(type);

      if (yoFramePointEnabledPair == null)
      {
         String namePrefix = centerOfMassName;
         namePrefix += type.getName();
         namePrefix += Space.POSITION.getName();
         YoMutableFramePoint3D yoFramePoint = new YoMutableFramePoint3D(namePrefix, "", registry, worldFrame);
         List<YoBoolean> endabledList = new ArrayList<>();
         yoFramePointEnabledPair = new ImmutablePair<>(yoFramePoint, endabledList);
         centerOfMassPositions.put(type, yoFramePointEnabledPair);
         clearableData.add(yoFramePointEnabledPair);
      }

      yoFramePointEnabledPair.getRight().add(enabled);

      return yoFramePointEnabledPair.getLeft();
   }

   /**
    * Retrieves and returns the {@code YoMutableFrameVector3D} for the center of mass associated with the given
    * {@code type}, and {@code space}, if it does not exist it is created.
    *
    * @param type the type of the data to retrieve.
    * @param space the space of the data to retrieve.
    * @return the unique {@code YoMutableFrameVector3D} matching the search criteria.
    */
   public YoMutableFrameVector3D getCenterOfMassDataVector(Type type, Space space, YoBoolean enabled)
   {
      EnumMap<Space, Pair<YoMutableFrameVector3D, List<YoBoolean>>> dataVectors = centerOfMassDataVectors.get(type);

      if (dataVectors == null)
      {
         dataVectors = new EnumMap<>(Space.class);
         centerOfMassDataVectors.put(type, dataVectors);
      }

      Pair<YoMutableFrameVector3D, List<YoBoolean>> yoFrameVectorEnabledPair = dataVectors.get(space);

      if (yoFrameVectorEnabledPair == null)
      {
         String namePrefix = centerOfMassName;
         namePrefix += type.getName();
         namePrefix += space.getName();
         YoMutableFrameVector3D yoFrameVector = new YoMutableFrameVector3D(namePrefix, "", registry, worldFrame);
         List<YoBoolean> endabledList = new ArrayList<>();
         yoFrameVectorEnabledPair = new ImmutablePair<>(yoFrameVector, endabledList);
         dataVectors.put(space, yoFrameVectorEnabledPair);
         clearableData.add(yoFrameVectorEnabledPair);
      }

      yoFrameVectorEnabledPair.getRight().add(enabled);

      return yoFrameVectorEnabledPair.getLeft();
   }

   /**
    * Retrieves and returns the {@code AlphaFilteredYoMutableFrameVector3D} for the center of mass associated
    * with the given {@code type} and {@code space}, if it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt} and {@code breakFrequencyProvider} are only used if the data does
    * not exist yet.
    * </p>
    *
    * @param space the space of the data to retrieve.
    * @param rawDataType the type of the raw vector onto which the rate limit is to be applied.
    * @param dt the duration of a control tick.
    * @param breakFrequencyProvider the break frequency to use for the low-pass filter. Not modified.
    * @return the unique {@code AlphaFilteredYoMutableFrameVector3D} matching the search criteria.
    */
   public AlphaFilteredYoMutableFrameVector3D getCenterOfMassAlphaFilteredDataVector(Type rawDataType, Space space, double dt,
                                                                                     DoubleProvider breakFrequencyProvider, YoBoolean enabled)
   {
      Pair<AlphaFilteredYoMutableFrameVector3D, List<YoBoolean>> alphaFilteredYoFrameVectorEnabledPair = centerOfMassFilteredDataVectors.get(space);

      if (alphaFilteredYoFrameVectorEnabledPair == null)
      {
         String namePrefix = centerOfMassName;
         namePrefix += "Filtered";
         namePrefix += rawDataType.getName();
         namePrefix += space.getName();
         FrameVector3DReadOnly rawYoFrameVector = getCenterOfMassDataVector(rawDataType, space, enabled);
         DoubleProvider alpha = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequencyProvider.getValue(), dt);
         AlphaFilteredYoMutableFrameVector3D alphaFilteredYoFrameVector = new AlphaFilteredYoMutableFrameVector3D(namePrefix, "", registry, alpha, rawYoFrameVector);
         List<YoBoolean> endabledList = new ArrayList<>();
         alphaFilteredYoFrameVectorEnabledPair = new ImmutablePair<>(alphaFilteredYoFrameVector, endabledList);
         centerOfMassFilteredDataVectors.put(space, alphaFilteredYoFrameVectorEnabledPair);
         clearableData.add(alphaFilteredYoFrameVectorEnabledPair);
      }

      alphaFilteredYoFrameVectorEnabledPair.getRight().add(enabled);

      return alphaFilteredYoFrameVectorEnabledPair.getLeft();
   }

   /**
    * Retrieves and returns the {@code RateLimitedYoMutableFrameVector3D} for the center of mass associated with
    * the given {@code type}, and {@code space}, if it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt} and {@code maximumRate} are only used if the data does not exist
    * yet.
    * </p>
    *
    * @param space the space of the data to retrieve.
    * @param rawDataType the type of the raw vector onto which the rate limit is to be applied.
    * @param dt the duration of a control tick.
    * @param maximumRate the maximum rate allowed rate. Not modified.
    * @return the unique {@code RateLimitedYoMutableFrameVector3D} matching the search criteria.
    */
   public RateLimitedYoMutableFrameVector3D getCenterOfMassRateLimitedDataVector(Type rawDataType, Space space, double dt, YoDouble maximumRate,
                                                                                 YoBoolean enabled)
   {
      Pair<RateLimitedYoMutableFrameVector3D, List<YoBoolean>> rateLimitedYoFrameVectorEnabledPair = centerOfMassRateLimitedDataVectors.get(space);

      if (rateLimitedYoFrameVectorEnabledPair == null)
      {
         String namePrefix = centerOfMassName;
         namePrefix += "RateLimited";
         namePrefix += rawDataType.getName();
         namePrefix += space.getName();
         FrameVector3DReadOnly rawYoFrameVector = getCenterOfMassDataVector(rawDataType, space, enabled);
         RateLimitedYoMutableFrameVector3D rateLimitedYoFrameVector = new RateLimitedYoMutableFrameVector3D(namePrefix, "", registry, maximumRate, dt,
                                                                                                            rawYoFrameVector);
         List<YoBoolean> endabledList = new ArrayList<>();
         rateLimitedYoFrameVectorEnabledPair = new ImmutablePair<>(rateLimitedYoFrameVector, endabledList);
         centerOfMassRateLimitedDataVectors.put(space, rateLimitedYoFrameVectorEnabledPair);
         clearableData.add(rateLimitedYoFrameVectorEnabledPair);
      }

      rateLimitedYoFrameVectorEnabledPair.getRight().add(enabled);

      return rateLimitedYoFrameVectorEnabledPair.getLeft();
   }

   /**
    * Retrieves and returns the set of gains {@code YoPositionPIDGainsInterface} for the center of
    * mass, if it does not exist it is created.
    *
    * @param useIntegrator whether to create the gains necessary to compute the integral term.
    * @return the unique {@code YoPositionPIDGainsInterface} for the center of mass.
    */
   public YoPID3DGains getCenterOfMassGains(boolean useIntegrator)
   {
      if (centerOfMassPositionGains == null)
      {
         centerOfMassPositionGains = new DefaultYoPID3DGains(centerOfMassName, GainCoupling.NONE, useIntegrator, registry);
      }
      return centerOfMassPositionGains;
   }

   /**
    * Retrieves and returns the {@code YoMutableFramePoint3D} associated with the given end-effector and
    * {@code type}, if it does not exist it is created.
    * <p>
    * The name prefix of the created variable is created as follows:<br>
    * {@code namePrefix = endEffector.getName() + type.getName() +}
    * {@link Space#POSITION}{@code .getName()}<br>
    * Such that the desired position for the rigid-body 'rightHand' will have the prefix:
    * "rightHandDesiredPosition".
    * </p>
    *
    * @param endEffector the end-effector to which the returned data is associated.
    * @param type the type of the data to retrieve.
    * @return the unique {@code YoMutableFramePoint3D} matching the search criteria.
    */
   public YoMutableFramePoint3D getPosition(RigidBodyBasics endEffector, Type type, YoBoolean enabled)
   {
      EnumMap<Type, Pair<YoMutableFramePoint3D, List<YoBoolean>>> typeDependentPositions = endEffectorPositions.get(endEffector);

      if (typeDependentPositions == null)
      {
         typeDependentPositions = new EnumMap<>(Type.class);
         endEffectorPositions.put(endEffector, typeDependentPositions);
      }

      Pair<YoMutableFramePoint3D, List<YoBoolean>> yoFramePointEnabledPair = typeDependentPositions.get(type);

      if (yoFramePointEnabledPair == null)
      {
         String namePrefix = endEffector.getName();
         namePrefix += type.getName();
         namePrefix += Space.POSITION.getName();
         YoMutableFramePoint3D yoFramePoint = new YoMutableFramePoint3D(namePrefix, "", registry, worldFrame);
         List<YoBoolean> endabledList = new ArrayList<>();
         yoFramePointEnabledPair = new ImmutablePair<>(yoFramePoint, endabledList);
         typeDependentPositions.put(type, yoFramePointEnabledPair);
         clearableData.add(yoFramePointEnabledPair);
      }

      yoFramePointEnabledPair.getRight().add(enabled);

      return yoFramePointEnabledPair.getLeft();
   }

   /**
    * Retrieves and returns the {@code YoMutableFrameQuaternion} associated with the given end-effector and
    * {@code type}, if it does not exist it is created.
    * <p>
    * The name prefix of the created variable is created as follows:<br>
    * {@code namePrefix = endEffector.getName() + type.getName() +}
    * {@link Space#ORIENTATION}{@code .getName()}<br>
    * Such that the current orientation for the rigid-body 'rightHand' will have the prefix:
    * "rightHandCurrentOrientation".
    * </p>
    *
    * @param endEffector the end-effector to which the returned data is associated.
    * @param type the type of the data to retrieve.
    * @return the unique {@code YoMutableFrameQuaternion} matching the search criteria.
    */
   public YoMutableFrameQuaternion getOrientation(RigidBodyBasics endEffector, Type type, YoBoolean enabled)
   {
      EnumMap<Type, Pair<YoMutableFrameQuaternion, List<YoBoolean>>> typeDependentOrientations = endEffectorOrientations.get(endEffector);

      if (typeDependentOrientations == null)
      {
         typeDependentOrientations = new EnumMap<>(Type.class);
         endEffectorOrientations.put(endEffector, typeDependentOrientations);
      }

      Pair<YoMutableFrameQuaternion, List<YoBoolean>> yoFrameQuaternionEnabledPair = typeDependentOrientations.get(type);

      if (yoFrameQuaternionEnabledPair == null)
      {
         String namePrefix = endEffector.getName();
         namePrefix += type.getName();
         namePrefix += Space.ORIENTATION.getName();
         YoMutableFrameQuaternion yoFrameQuaternion = new YoMutableFrameQuaternion(namePrefix, "", registry, worldFrame);
         List<YoBoolean> endabledList = new ArrayList<>();
         yoFrameQuaternionEnabledPair = new ImmutablePair<>(yoFrameQuaternion, endabledList);
         typeDependentOrientations.put(type, yoFrameQuaternionEnabledPair);
         clearableData.add(yoFrameQuaternionEnabledPair);
      }

      yoFrameQuaternionEnabledPair.getRight().add(enabled);

      return yoFrameQuaternionEnabledPair.getLeft();
   }

   /**
    * Retrieves and returns the {@code YoMutableFrameVector3D} associated with the given end-effector,
    * {@code type}, and {@code space}, if it does not exist it is created.
    * <p>
    * The name prefix of the created variable is created as follows:<br>
    * {@code namePrefix = endEffector.getName() + type.getName() + space.getName()}<br>
    * Such that the desired linear velocity for the rigid-body 'rightHand' will have the prefix:
    * "rightHandDesiredLinearVelocity".
    * </p>
    *
    * @param endEffector the end-effector to which the returned data is associated.
    * @param type the type of the data to retrieve.
    * @param space the space of the data to retrieve.
    * @return the unique {@code YoMutableFrameVector3D} matching the search criteria.
    */
   public YoMutableFrameVector3D getDataVector(RigidBodyBasics endEffector, Type type, Space space, YoBoolean enabled)
   {
      EnumMap<Type, EnumMap<Space, Pair<YoMutableFrameVector3D, List<YoBoolean>>>> dataVectorStep1 = endEffectorDataVectors.get(endEffector);

      if (dataVectorStep1 == null)
      {
         dataVectorStep1 = new EnumMap<>(Type.class);
         endEffectorDataVectors.put(endEffector, dataVectorStep1);
      }

      EnumMap<Space, Pair<YoMutableFrameVector3D, List<YoBoolean>>> dataVectorStep2 = dataVectorStep1.get(type);

      if (dataVectorStep2 == null)
      {
         dataVectorStep2 = new EnumMap<>(Space.class);
         dataVectorStep1.put(type, dataVectorStep2);
      }

      Pair<YoMutableFrameVector3D, List<YoBoolean>> yoFrameVectorEnabledPair = dataVectorStep2.get(space);

      if (yoFrameVectorEnabledPair == null)
      {
         String namePrefix = endEffector.getName();
         namePrefix += type.getName();
         namePrefix += space.getName();
         YoMutableFrameVector3D yoFrameVector = new YoMutableFrameVector3D(namePrefix, "", registry, worldFrame);
         List<YoBoolean> endabledList = new ArrayList<>();
         yoFrameVectorEnabledPair = new ImmutablePair<>(yoFrameVector, endabledList);
         dataVectorStep2.put(space, yoFrameVectorEnabledPair);
         clearableData.add(yoFrameVectorEnabledPair);
      }

      yoFrameVectorEnabledPair.getRight().add(enabled);

      return yoFrameVectorEnabledPair.getLeft();
   }

   /**
    * Retrieves and returns the {@code RateLimitedYoMutableFrameVector3D} associated with the given
    * end-effector, {@code type}, and {@code space}, if it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt} and {@code maximumRate} are only used if the data does not exist
    * yet.
    * </p>
    * <p>
    * The name prefix of the created variable is created as follows:<br>
    * {@code namePrefix = endEffector.getName() + "RateLimited" + rawDataType.getName() + space.getName()}<br>
    * Such that the rate-limited vector of the desired linear acceleration for the rigid-body
    * 'rightHand' will have the prefix: "rightHandRateLimitedDesiredLinearAcceleration".
    * </p>
    *
    * @param endEffector the end-effector to which the returned data is associated.
    * @param space the space of the data to retrieve.
    * @param rawDataType the type of the raw vector onto which the rate limit is to be applied.
    * @param dt the duration of a control tick.
    * @param maximumRate the maximum rate allowed rate. Not modified.
    * @return the unique {@code RateLimitedYoMutableFrameVector3D} matching the search criteria.
    */
   public RateLimitedYoMutableFrameVector3D getRateLimitedDataVector(RigidBodyBasics endEffector, Type rawDataType, Space space, double dt,
                                                                     YoDouble maximumRate, YoBoolean enabled)
   {
      EnumMap<Space, Pair<RateLimitedYoMutableFrameVector3D, List<YoBoolean>>> endEffectorDataVectors = endEffectorRateLimitedDataVectors.get(endEffector);

      if (endEffectorDataVectors == null)
      {
         endEffectorDataVectors = new EnumMap<>(Space.class);
         endEffectorRateLimitedDataVectors.put(endEffector, endEffectorDataVectors);
      }

      Pair<RateLimitedYoMutableFrameVector3D, List<YoBoolean>> rateLimitedYoFrameVectorEnabledPair = endEffectorDataVectors.get(space);

      if (rateLimitedYoFrameVectorEnabledPair == null)
      {
         String namePrefix = endEffector.getName();
         namePrefix += "RateLimited";
         namePrefix += rawDataType.getName();
         namePrefix += space.getName();
         FrameVector3DReadOnly rawYoFrameVector = getDataVector(endEffector, rawDataType, space, enabled);
         RateLimitedYoMutableFrameVector3D rateLimitedYoFrameVector = new RateLimitedYoMutableFrameVector3D(namePrefix, "", registry, maximumRate, dt,
                                                                                                            rawYoFrameVector);
         List<YoBoolean> endabledList = new ArrayList<>();
         rateLimitedYoFrameVectorEnabledPair = new ImmutablePair<>(rateLimitedYoFrameVector, endabledList);
         endEffectorDataVectors.put(space, rateLimitedYoFrameVectorEnabledPair);
         clearableData.add(rateLimitedYoFrameVectorEnabledPair);
      }

      rateLimitedYoFrameVectorEnabledPair.getRight().add(enabled);

      return rateLimitedYoFrameVectorEnabledPair.getLeft();
   }

   /**
    * Retrieves and returns the {@code AlphaFilteredYoMutableFrameVector3D} associated with the given
    * end-effector, {@code type}, and {@code space}, if it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt} and {@code breakFrequencyProvider} are only used if the data does
    * not exist yet.
    * </p>
    * <p>
    * The name prefix of the created variable is created as follows:<br>
    * {@code namePrefix = endEffector.getName() + "Filtered" + rawDataType.getName() + space.getName()}<br>
    * Such that the filtered vector of the linear velocity error for the rigid-body 'rightHand' will
    * have the prefix: "rightHandFilteredErrorLinearVelocity".
    * </p>
    *
    * @param endEffector the end-effector to which the returned data is associated.
    * @param space the space of the data to retrieve.
    * @param rawDataType the type of the raw vector onto which the rate limit is to be applied.
    * @param dt the duration of a control tick.
    * @param breakFrequencyProvider the break frequency to use for the low-pass filter. Not modified.
    * @return the unique {@code AlphaFilteredYoMutableFrameVector3D} matching the search criteria.
    */
   public AlphaFilteredYoMutableFrameVector3D getAlphaFilteredDataVector(RigidBodyBasics endEffector, Type rawDataType, Space space, double dt,
                                                                         DoubleProvider breakFrequencyProvider, YoBoolean enabled)
   {
      EnumMap<Space, Pair<AlphaFilteredYoMutableFrameVector3D, List<YoBoolean>>> endEffectorDataVectors = endEffectorFilteredDataVectors.get(endEffector);

      if (endEffectorDataVectors == null)
      {
         endEffectorDataVectors = new EnumMap<>(Space.class);
         endEffectorFilteredDataVectors.put(endEffector, endEffectorDataVectors);
      }

      Pair<AlphaFilteredYoMutableFrameVector3D, List<YoBoolean>> alphaFilteredYoFrameVectorEnabledPair = endEffectorDataVectors.get(space);

      if (alphaFilteredYoFrameVectorEnabledPair == null)
      {
         String namePrefix = endEffector.getName();
         namePrefix += "Filtered";
         namePrefix += rawDataType.getName();
         namePrefix += space.getName();
         FrameVector3DReadOnly rawYoFrameVector = getDataVector(endEffector, rawDataType, space, enabled);
         DoubleProvider alpha = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequencyProvider.getValue(), dt);
         AlphaFilteredYoMutableFrameVector3D alphaFilteredYoFrameVector = new AlphaFilteredYoMutableFrameVector3D(namePrefix, "", registry, alpha, rawYoFrameVector);
         List<YoBoolean> endabledList = new ArrayList<>();
         alphaFilteredYoFrameVectorEnabledPair = new ImmutablePair<>(alphaFilteredYoFrameVector, endabledList);
         endEffectorDataVectors.put(space, alphaFilteredYoFrameVectorEnabledPair);
         clearableData.add(alphaFilteredYoFrameVectorEnabledPair);
      }

      alphaFilteredYoFrameVectorEnabledPair.getRight().add(enabled);

      return alphaFilteredYoFrameVectorEnabledPair.getLeft();
   }

   /**
    * Retrieves and returns the {@code YoMutableFramePose3D} associated with the given
    * end-effector and {@code type}, if it does not exist it is created.
    *
    * @param endEffector the end-effector to which the returned data is associated.
    * @param type the type of the data to retrieve.
    * @return the unique {@code YoMutableFramePose3D} matching the search criteria.
    */
   public YoMutableFramePose3D getPose(RigidBodyBasics endEffector, Type type, YoBoolean enabled)
   {
      return new YoMutableFramePose3D(getPosition(endEffector, type, enabled), getOrientation(endEffector, type, enabled));
   }

   /**
    * Retrieves and returns the {@code YoMutableFrameSpatialVector} for holding the angular and linear velocities
    * of the given end-effector for representing a given data {@code type}. If it does not exist it is
    * created.
    *
    * @param endEffector the end-effector to which the returned data is associated.
    * @param type the type of the data to retrieve.
    * @return the unique {@code YoMutableFrameSpatialVector} matching the search criteria.
    */
   public YoMutableFrameSpatialVector getVelocity(RigidBodyBasics endEffector, Type type, YoBoolean enabled)
   {
      return new YoMutableFrameSpatialVector(getDataVector(endEffector, type, Space.ANGULAR_VELOCITY, enabled),
                                             getDataVector(endEffector, type, Space.LINEAR_VELOCITY, enabled));
   }

   /**
    * Retrieves and returns the {@code YoMutableFrameSpatialVector} for holding the angular and linear
    * accelerations of the given end-effector for representing a given data {@code type}. If it does
    * not exist it is created.
    *
    * @param endEffector the end-effector to which the returned data is associated.
    * @param type the type of the data to retrieve.
    * @return the unique {@code YoMutableFrameSpatialVector} matching the search criteria.
    */
   public YoMutableFrameSpatialVector getAcceleration(RigidBodyBasics endEffector, Type type, YoBoolean enabled)
   {
      return new YoMutableFrameSpatialVector(getDataVector(endEffector, type, Space.ANGULAR_ACCELERATION, enabled),
                                             getDataVector(endEffector, type, Space.LINEAR_ACCELERATION, enabled));
   }

   /**
    * Retrieves and returns the {@code YoMutableFrameSpatialVector} for holding the angular and linear forces of
    * the given end-effector for representing a given data {@code type}. If it does not exist it is
    * created.
    *
    * @param endEffector the end-effector to which the returned data is associated.
    * @param type the type of the data to retrieve.
    * @return the unique {@code YoMutableFrameSpatialVector} matching the search criteria.
    */
   public YoMutableFrameSpatialVector getWrench(RigidBodyBasics endEffector, Type type, YoBoolean enabled)
   {
      return new YoMutableFrameSpatialVector(getDataVector(endEffector, type, Space.ANGULAR_TORQUE, enabled),
                                             getDataVector(endEffector, type, Space.LINEAR_FORCE, enabled));
   }

   /**
    * Retrieves and returns the {@code RateLimitedYoMutableSpatialVector} for the rate-limited angular and
    * linear velocities of the given end-effector. The data type of the vector is defined by
    * {@code type}. If it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt}, {@code maximumLinearRate}, and {@code maximumAngularRate} are
    * only used if the data does not exist yet.
    * </p>
    *
    * @param endEffector the end-effector to which the returned data is associated.
    * @param rawDataType the type of the raw vector onto which the rate limit is to be applied.
    * @param dt the duration of a control tick.
    * @param maximumAngularRate the maximum angular rate allowed rate. Not modified.
    * @param maximumLinearRate the maximum linear rate allowed rate. Not modified.
    * @return the unique {@code RateLimitedYoMutableSpatialVector} matching the search criteria.
    */
   public RateLimitedYoMutableSpatialVector getRateLimitedVelocity(RigidBodyBasics endEffector, Type rawDataType, double dt, YoDouble maximumAngularRate,
                                                                   YoDouble maximumLinearRate, YoBoolean enabled)
   {
      return new RateLimitedYoMutableSpatialVector(getRateLimitedDataVector(endEffector, rawDataType, Space.ANGULAR_VELOCITY, dt, maximumAngularRate, enabled),
                                                   getRateLimitedDataVector(endEffector, rawDataType, Space.LINEAR_VELOCITY, dt, maximumLinearRate, enabled));
   }

   /**
    * Retrieves and returns the {@code AlphaFilteredYoMutableFrameSpatialVector} for the filtered angular and
    * linear velocity errors of the given end-effector. If it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt}, {@code breakFrequencyLinearPart}, and
    * {@code breakFrequencyAngularPart} are only used if the data does not exist yet.
    * </p>
    *
    * @param endEffector the end-effector to which the returned data is associated.
    * @param rawDataType the type of the raw vector onto which the filter is to be applied.
    * @param dt the duration of a control tick.
    * @param breakFrequencyAngularPart the break frequency to use for the angular part of the velocity
    *           error. Not modified.
    * @param breakFrequencyLinearPart the break frequency to use for the linear part of the velocity
    *           error. Not modified.
    * @return the unique {@code AlphaFilteredYoMutableFrameSpatialVector} matching the search criteria.
    */
   public AlphaFilteredYoMutableFrameSpatialVector getAlphaFilteredVelocity(RigidBodyBasics endEffector, Type rawDataType, double dt,
                                                                            DoubleProvider breakFrequencyAngularPart, DoubleProvider breakFrequencyLinearPart,
                                                                            YoBoolean enabled)
   {
      return new AlphaFilteredYoMutableFrameSpatialVector(getAlphaFilteredDataVector(endEffector, rawDataType, Space.ANGULAR_VELOCITY, dt,
                                                                                     breakFrequencyAngularPart, enabled),
                                                          getAlphaFilteredDataVector(endEffector, rawDataType, Space.LINEAR_VELOCITY, dt,
                                                                                     breakFrequencyLinearPart, enabled));
   }

   /**
    * Retrieves and returns the {@code RateLimitedYoMutableSpatialVector} for the rate-limited angular and
    * linear accelerations of the given end-effector. The data type of the vector is defined by
    * {@code type}. If it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt}, {@code maximumLinearRate}, and {@code maximumAngularRate} are
    * only used if the data does not exist yet.
    * </p>
    *
    * @param endEffector the end-effector to which the returned data is associated.
    * @param rawDataType the type of the raw vector onto which the rate limit is to be applied.
    * @param dt the duration of a control tick.
    * @param maximumAngularRate the maximum angular rate allowed rate. Not modified.
    * @param maximumLinearRate the maximum linear rate allowed rate. Not modified.
    * @return the unique {@code RateLimitedYoMutableSpatialVector} matching the search criteria.
    */
   public RateLimitedYoMutableSpatialVector getRateLimitedAcceleration(RigidBodyBasics endEffector, Type rawDataType, double dt, YoDouble maximumAngularRate,
                                                                       YoDouble maximumLinearRate, YoBoolean enabled)
   {
      return new RateLimitedYoMutableSpatialVector(getRateLimitedDataVector(endEffector, rawDataType, Space.ANGULAR_ACCELERATION, dt, maximumAngularRate,
                                                                            enabled),
                                                   getRateLimitedDataVector(endEffector, rawDataType, Space.LINEAR_ACCELERATION, dt, maximumLinearRate,
                                                                            enabled));
   }

   /**
    * Retrieves and returns the {@code RateLimitedYoMutableSpatialVector} for the rate-limited angular and
    * linear accelerations of the given end-effector. The date type of the vector is defined by
    * {@code type}. If it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt}, {@code maximumLinearRate}, and {@code maximumAngularRate} are
    * only used if the data does not exist yet.
    * </p>
    *
    * @param endEffector the end-effector to which the returned data is associated.
    * @param rawDataType the type of the raw vector onto which the rate limit is to be applied.
    * @param dt the duration of a control tick.
    * @param maximumAngularRate the maximum angular rate allowed rate. Not modified.
    * @param maximumLinearRate the maximum linear rate allowed rate. Not modified.
    * @return the unique {@code RateLimitedYoMutableSpatialVector} matching the search criteria.
    */
   public RateLimitedYoMutableSpatialVector getRateLimitedWrench(RigidBodyBasics endEffector, Type rawDataType, double dt, YoDouble maximumAngularRate,
                                                                 YoDouble maximumLinearRate, YoBoolean enabled)
   {
      return new RateLimitedYoMutableSpatialVector(getRateLimitedDataVector(endEffector, rawDataType, Space.ANGULAR_TORQUE, dt, maximumAngularRate, enabled),
                                                   getRateLimitedDataVector(endEffector, rawDataType, Space.LINEAR_FORCE, dt, maximumLinearRate, enabled));
   }

   /**
    * Retrieves and returns the set of orientation gains {@code YoPID3DGains} associated to the given
    * end-effector, if it does not exist it is created.
    *
    * @param endEffector the end-effector to which the gains are associated.
    * @param useIntegrator whether to create the gains necessary to compute the integral term.
    * @return the unique {@code YoPID3DGains} associated with the given end-effector.
    */
   public YoPID3DGains getOrientationGains(RigidBodyBasics endEffector, boolean useIntegrator)
   {
      YoPID3DGains gains = endEffectorOrientationGains.get(endEffector);

      if (gains == null)
      {
         gains = new DefaultYoPID3DGains(endEffector.getName() + "Orientation", GainCoupling.NONE, useIntegrator, registry);
         endEffectorOrientationGains.put(endEffector, gains);
      }
      return gains;
   }

   /**
    * Retrieves and returns the set of position gains {@code YoPID3DGains} associated to the given
    * end-effector, if it does not exist it is created.
    *
    * @param endEffector the end-effector to which the gains are associated.
    * @param useIntegrator whether to create the gains necessary to compute the integral term.
    * @return the unique {@code YoPID3DGains} associated with the given end-effector.
    */
   public YoPID3DGains getPositionGains(RigidBodyBasics endEffector, boolean useIntegrator)
   {
      YoPID3DGains gains = endEffectorPositionGains.get(endEffector);

      if (gains == null)
      {
         gains = new DefaultYoPID3DGains(endEffector.getName() + "Position", GainCoupling.NONE, useIntegrator, registry);
         endEffectorPositionGains.put(endEffector, gains);
      }
      return gains;
   }

   /**
    * Retrieves and returns the set of gains {@code YoPIDSE3Gains} associated to the given
    * end-effector, if it does not exist it is created.
    *
    * @param endEffector the end-effector to which the gains are associated.
    * @param useIntegrator whether to create the gains necessary to compute the integral term.
    * @return the unique {@code YoPIDSE3Gains} associated with the given end-effector.
    */
   public YoPIDSE3Gains getSE3PIDGains(RigidBodyBasics endEffector, boolean useIntegrator)
   {
      YoPID3DGains positionGains = getPositionGains(endEffector, useIntegrator);
      YoPID3DGains orientationGains = getOrientationGains(endEffector, useIntegrator);
      return new DefaultYoPIDSE3Gains(positionGains, orientationGains);
   }

   /**
    * Retrieves and returns the control frame {@code YoSE3OffsetFrame} associated to the given
    * end-effector, if it does not exist it is created.
    *
    * @param endEffector the end-effector to which the control frame is associated.
    * @return the unique {@code YoSE3OffsetFrame} control frame associated with the given end-effector.
    */
   public YoSE3OffsetFrame getControlFrame(RigidBodyBasics endEffector)
   {
      YoSE3OffsetFrame controlFrame = endEffectorControlFrames.get(endEffector);

      if (controlFrame == null)
      {
         controlFrame = new YoSE3OffsetFrame(endEffector.getName() + "BodyFixedControlFrame", endEffector.getBodyFixedFrame(), registry);
         endEffectorControlFrames.put(endEffector, controlFrame);
      }

      return controlFrame;
   }

   /**
    * Calls {@link Clearable#setToNaN()} to all the register objects used by the feedback controllers.
    * <p>
    * The method should be called at the beginning of the controller core tick such that the unused
    * part of the data will be {@link Double#NaN} making it clear what it is used and what is not.
    * </p>
    */
   public void clearUnusedData()
   {
      for (int i = 0; i < clearableData.size(); i++)
      {
         Pair<? extends Clearable, List<YoBoolean>> pair = clearableData.get(i);
         if (!hasData(pair.getRight()))
            pair.getLeft().setToNaN();
      }
   }

   @Override
   public boolean getCenterOfMassPositionData(FramePoint3DBasics positionDataToPack, Type type)
   {
      Pair<YoMutableFramePoint3D, List<YoBoolean>> positionData = centerOfMassPositions.get(type);

      if (positionData == null || !hasData(positionData.getRight()))
         return false;

      positionDataToPack.setIncludingFrame(positionData.getLeft());
      return true;
   }

   @Override
   public boolean getCenterOfMassVectorData(FrameVector3DBasics vectorDataToPack, Type type, Space space)
   {
      EnumMap<Space, Pair<YoMutableFrameVector3D, List<YoBoolean>>> endEffectorDataTyped = centerOfMassDataVectors.get(type);

      if (endEffectorDataTyped == null)
         return false;

      Pair<YoMutableFrameVector3D, List<YoBoolean>> vectorData = endEffectorDataTyped.get(space);

      if (vectorData == null || !hasData(vectorData.getRight()))
         return false;

      vectorDataToPack.setIncludingFrame(vectorData.getLeft());
      return true;
   }

   @Override
   public boolean getPositionData(RigidBodyBasics endEffector, FramePoint3DBasics positionDataToPack, Type type)
   {
      EnumMap<Type, Pair<YoMutableFramePoint3D, List<YoBoolean>>> endEffectorData = endEffectorPositions.get(endEffector);

      if (endEffectorData == null)
         return false;

      Pair<YoMutableFramePoint3D, List<YoBoolean>> positionData = endEffectorData.get(type);

      if (positionData == null || !hasData(positionData.getRight()))
         return false;

      positionDataToPack.setIncludingFrame(positionData.getLeft());
      return true;
   }

   @Override
   public boolean getOrientationData(RigidBodyBasics endEffector, FrameQuaternionBasics orientationDataToPack, Type type)
   {
      EnumMap<Type, Pair<YoMutableFrameQuaternion, List<YoBoolean>>> endEffectorData = endEffectorOrientations.get(endEffector);

      if (endEffectorData == null)
         return false;

      Pair<YoMutableFrameQuaternion, List<YoBoolean>> orientationData = endEffectorData.get(type);

      if (orientationData == null || !hasData(orientationData.getRight()))
         return false;

      orientationDataToPack.setIncludingFrame(orientationData.getLeft());
      return true;
   }

   @Override
   public boolean getVectorData(RigidBodyBasics endEffector, FrameVector3DBasics vectorDataToPack, Type type, Space space)
   {
      EnumMap<Type, EnumMap<Space, Pair<YoMutableFrameVector3D, List<YoBoolean>>>> endEffectorData = endEffectorDataVectors.get(endEffector);

      if (endEffectorData == null)
         return false;

      EnumMap<Space, Pair<YoMutableFrameVector3D, List<YoBoolean>>> endEffectorDataTyped = endEffectorData.get(type);

      if (endEffectorDataTyped == null)
         return false;

      Pair<YoMutableFrameVector3D, List<YoBoolean>> vectorData = endEffectorDataTyped.get(space);

      if (vectorData == null || !hasData(vectorData.getRight()))
         return false;

      vectorDataToPack.setIncludingFrame(vectorData.getLeft());
      return true;
   }

   private static boolean hasData(List<YoBoolean> enabledList)
   {
      for (int i = 0; i < enabledList.size(); i++)
      {
         if (enabledList.get(i).getBooleanValue())
            return true;
      }
      return false;
   }

   public DoubleProvider getErrorVelocityFilterBreakFrequency(String endEffectorOrJointName)
   {
      return errorVelocityFilterBreakFrequencies.get(endEffectorOrJointName);
   }
}
