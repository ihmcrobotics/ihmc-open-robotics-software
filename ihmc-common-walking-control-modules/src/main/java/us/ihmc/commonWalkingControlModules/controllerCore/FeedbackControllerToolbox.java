package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.controlModules.YoOrientationFrame;
import us.ihmc.commonWalkingControlModules.controlModules.YoSE3OffsetFrame;
import us.ihmc.commonWalkingControlModules.controlModules.YoTranslationFrame;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBAlphaFilteredVector3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBAlphaFilteredVector6D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBPoint3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBPose3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBQuaternion3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBRateLimitedVector3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBRateLimitedVector6D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBVector3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBVector6D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FeedbackControllerData;
import us.ihmc.commonWalkingControlModules.controllerCore.data.SpaceData3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.SpaceData6D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.Type;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

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
 */
public class FeedbackControllerToolbox implements FeedbackControllerDataHolderReadOnly
{
   public static final String centerOfMassName = "centerOfMass";

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private SingleFeedbackControllerDataPool centerOfMassDataPool;
   private final Map<RigidBodyBasics, List<SingleFeedbackControllerDataPool>> endEffectorDataPoolMap = new HashMap<>();
   private final List<SingleFeedbackControllerDataPool> singleFeedbackControllerDataPoolList = new ArrayList<>();

   private final Map<String, DoubleProvider> errorVelocityFilterBreakFrequencies;

   private final InverseDynamicsCommandList lastFeedbackControllerInverseDynamicsOutput = new InverseDynamicsCommandList();
   private final InverseKinematicsCommandList lastFeedbackControllerInverseKinematicsOutput = new InverseKinematicsCommandList();
   private final VirtualModelControlCommandList lastFeedbackControllerVirtualModelControlOutput = new VirtualModelControlCommandList();

   public FeedbackControllerToolbox(YoRegistry parentRegistry)
   {
      this(FeedbackControllerSettings.getDefault(), parentRegistry);
   }

   public FeedbackControllerToolbox(FeedbackControllerSettings settings, YoRegistry parentRegistry)
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

   public YoRegistry getRegistry()
   {
      return registry;
   }

   /**
    * Stores the feedback controllers' output for later access to the user.
    * 
    * @param output the output of the controller.
    */
   public void registerFeedbackControllerOutput(InverseDynamicsCommandList output)
   {
      lastFeedbackControllerInverseDynamicsOutput.set(output);
      lastFeedbackControllerInverseKinematicsOutput.clear();
      lastFeedbackControllerVirtualModelControlOutput.clear();
   }

   /**
    * Stores the feedback controllers' output for later access to the user.
    * 
    * @param output the output of the controller.
    */
   public void registerFeedbackControllerOutput(InverseKinematicsCommandList output)
   {
      lastFeedbackControllerInverseDynamicsOutput.clear();
      lastFeedbackControllerInverseKinematicsOutput.set(output);
      lastFeedbackControllerVirtualModelControlOutput.clear();
   }

   /**
    * Stores the feedback controllers' output for later access to the user.
    * 
    * @param output the output of the controller.
    */
   public void registerFeedbackControllerOutput(VirtualModelControlCommandList output)
   {
      lastFeedbackControllerInverseDynamicsOutput.clear();
      lastFeedbackControllerInverseKinematicsOutput.clear();
      lastFeedbackControllerVirtualModelControlOutput.set(output);
   }

   private SingleFeedbackControllerDataPool getOrCreateCenterOfMassDataPool()
   {
      if (centerOfMassDataPool == null)
      {
         centerOfMassDataPool = new SingleFeedbackControllerDataPool(centerOfMassName, 0, registry);
         singleFeedbackControllerDataPoolList.add(centerOfMassDataPool);
      }
      return centerOfMassDataPool;
   }

   /**
    * Retrieves and returns the {@code FBPoint3D} for the center of mass associated with the given
    * {@code type}, if it does not exist it is created.
    *
    * @param type               the type of the data to retrieve.
    * @param activeFlag         boolean provider that should reflect the active state of the controller
    *                           calling this factory. Used to determine whether the returned data is
    *                           up-to-date or not at runtime.
    * @param isRequiredVariable the {@code YoVariable}s do not get attached to this registry when the
    *                           variable is not a required variable and
    *                           {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                           {@code true}.
    * @return the unique {@code FBPoint3D} matching the search criterion.
    */
   public FBPoint3D getOrCreateCenterOfMassPositionData(Type type, BooleanProvider activeFlag, boolean isRequiredVariable)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateCenterOfMassDataPool();
      FBPoint3D positionData = dataPool.getOrCreatePositionData(type, isRequiredVariable);
      positionData.addActiveFlag(activeFlag);
      return positionData;
   }

   /**
    * Retrieves and returns the {@code FBVector3D} for the center of mass associated with the given
    * {@code type}, and {@code space}, if it does not exist it is created.
    *
    * @param type               the type of the data to retrieve.
    * @param space              the space of the data to retrieve.
    * @param activeFlag         boolean provider that should reflect the active state of the controller
    *                           calling this factory. Used to determine whether the returned data is
    *                           up-to-date or not at runtime.
    * @param isRequiredVariable the {@code YoVariable}s do not get attached to this registry when the
    *                           variable is not a required variable and
    *                           {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                           {@code true}.
    * @return the unique {@code FBVector3D} matching the search criteria.
    */
   public FBVector3D getOrCreateCenterOfMassVectorData(Type type, SpaceData3D space, BooleanProvider activeFlag, boolean isRequiredVariable)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateCenterOfMassDataPool();
      FBVector3D vectorData = dataPool.getOrCreateVectorData3D(type, space, isRequiredVariable);
      vectorData.addActiveFlag(activeFlag);
      return vectorData;
   }

   /**
    * Retrieves and returns the {@code FBAlphaFilteredVector3D} for the center of mass associated with
    * the given {@code type} and {@code space}, if it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt} and {@code breakFrequencyProvider} are only used if the data does
    * not exist yet.
    * </p>
    *
    * @param space                  the space of the data to retrieve.
    * @param rawDataType            the type of the raw vector onto which the rate limit is to be
    *                               applied.
    * @param dt                     the duration of a control tick.
    * @param breakFrequencyProvider the break frequency to use for the low-pass filter. Not modified.
    * @param activeFlag             boolean provider that should reflect the active state of the
    *                               controller calling this factory. Used to determine whether the
    *                               returned data is up-to-date or not at runtime.
    * @param isRequiredVariable     the {@code YoVariable}s do not get attached to this registry when
    *                               the variable is not a required variable and
    *                               {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                               {@code true}.
    * @return the unique {@code FBAlphaFilteredVector3D} matching the search criteria.
    */
   public FBAlphaFilteredVector3D getOrCreateCenterOfMassAlphaFilteredVectorData(Type rawDataType,
                                                                                 SpaceData3D space,
                                                                                 double dt,
                                                                                 DoubleProvider breakFrequencyProvider,
                                                                                 BooleanProvider activeFlag,
                                                                                 boolean isRequiredVariable)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateCenterOfMassDataPool();
      FBAlphaFilteredVector3D filteredVectorData = dataPool.getOrCreateAlphaFilteredVectorData(rawDataType,
                                                                                               space,
                                                                                               breakFrequencyProvider,
                                                                                               dt,
                                                                                               isRequiredVariable);
      filteredVectorData.addActiveFlag(activeFlag);
      return filteredVectorData;
   }

   /**
    * Retrieves and returns the {@code FBRateLimitedVector3D} for the center of mass associated with
    * the given {@code type}, and {@code space}, if it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt} and {@code maximumRate} are only used if the data does not exist
    * yet.
    * </p>
    *
    * @param space              the space of the data to retrieve.
    * @param rawDataType        the type of the raw vector onto which the rate limit is to be applied.
    * @param dt                 the duration of a control tick.
    * @param maximumRate        the maximum rate allowed rate. Not modified.
    * @param activeFlag         boolean provider that should reflect the active state of the controller
    *                           calling this factory. Used to determine whether the returned data is
    *                           up-to-date or not at runtime.
    * @param isRequiredVariable the {@code YoVariable}s do not get attached to this registry when the
    *                           variable is not a required variable and
    *                           {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                           {@code true}.
    * @return the unique {@code FBRateLimitedVector3D} matching the search criteria.
    */
   public FBRateLimitedVector3D getOrCreateCenterOfMassRateLimitedVectorData(Type rawDataType,
                                                                             SpaceData3D space,
                                                                             double dt,
                                                                             YoDouble maximumRate,
                                                                             BooleanProvider activeFlag,
                                                                             boolean isRequiredVariable)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateCenterOfMassDataPool();
      FBRateLimitedVector3D rateLimitedVectorData = dataPool.getOrCreateRateLimitedVectorData(rawDataType, space, maximumRate, dt, isRequiredVariable);
      rateLimitedVectorData.addActiveFlag(activeFlag);
      return rateLimitedVectorData;
   }

   /**
    * Retrieves and returns the set of gains {@code YoPID3DGains} for the center of mass, if it does
    * not exist it is created.
    *
    * @param useIntegrator      whether to create the gains necessary to compute the integral term.
    * @param isRequiredVariable the {@code YoVariable}s do not get attached to this registry when the
    *                           variable is not a required variable and
    *                           {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                           {@code true}.
    * @return the unique {@code YoPID3DGains} for the center of mass.
    */
   public YoPID3DGains getOrCreateCenterOfMassGains(boolean useIntegrator, boolean isRequiredVariable)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateCenterOfMassDataPool();
      return dataPool.getOrCreatePositionGains(useIntegrator, isRequiredVariable);
   }

   private SingleFeedbackControllerDataPool getOrCreateEndEffectorDataPool(RigidBodyBasics endEffector, int controllerIndex)
   {
      List<SingleFeedbackControllerDataPool> endEffectorDataPoolList = endEffectorDataPoolMap.get(endEffector);
      if (endEffectorDataPoolList == null)
      {
         endEffectorDataPoolList = new ArrayList<>();
         endEffectorDataPoolMap.put(endEffector, endEffectorDataPoolList);
      }

      while (endEffectorDataPoolList.size() <= controllerIndex)
      {
         SingleFeedbackControllerDataPool newPool = new SingleFeedbackControllerDataPool(endEffector.getName(), controllerIndex, registry);
         endEffectorDataPoolList.add(newPool);
         singleFeedbackControllerDataPoolList.add(newPool);
      }

      return endEffectorDataPoolList.get(controllerIndex);
   }

   /**
    * Retrieves and returns the {@code FBPoint3D} associated with the given end-effector and
    * {@code type}, if it does not exist it is created.
    * <p>
    * The name prefix of the created variable is created as follows:
    * 
    * <pre>
    * namePrefix = endEffector.getName() + type.getName() + SpaceData3D.POSITION.getName()
    * </pre>
    * 
    * Such that the desired position for the rigid-body 'rightHand' will have the prefix:
    * "rightHandDesiredPosition".
    * </p>
    *
    * @param endEffector        the end-effector to which the returned data is associated.
    * @param controllerIndex    the index of the feedback controller requesting the data.
    * @param type               the type of the data to retrieve.
    * @param activeFlag         boolean provider that should reflect the active state of the controller
    *                           calling this factory. Used to determine whether the returned data is
    *                           up-to-date or not at runtime.
    * @param isRequiredVariable the {@code YoVariable}s do not get attached to this registry when the
    *                           variable is not a required variable and
    *                           {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                           {@code true}.
    * @return the unique {@code FBPoint3D} matching the search criteria.
    */
   public FBPoint3D getOrCreatePositionData(RigidBodyBasics endEffector, int controllerIndex, Type type, BooleanProvider activeFlag, boolean isRequiredVariable)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateEndEffectorDataPool(endEffector, controllerIndex);
      FBPoint3D positionData = dataPool.getOrCreatePositionData(type, isRequiredVariable);
      positionData.addActiveFlag(activeFlag);
      return positionData;
   }

   /**
    * Retrieves and returns the {@code FBQuaternion3D} associated with the given end-effector and
    * {@code type}, if it does not exist it is created.
    * <p>
    * The name prefix of the created variable is created as follows:
    * 
    * <pre>
    * namePrefix = endEffector.getName() + type.getName() + SpaceData3D.ORIENTATION.getName()
    * </pre>
    * 
    * Such that the current orientation for the rigid-body 'rightHand' will have the prefix:
    * "rightHandCurrentOrientation".
    * </p>
    *
    * @param endEffector        the end-effector to which the returned data is associated.
    * @param controllerIndex    the index of the feedback controller requesting the data.
    * @param type               the type of the data to retrieve.
    * @param activeFlag         boolean provider that should reflect the active state of the controller
    *                           calling this factory. Used to determine whether the returned data is
    *                           up-to-date or not at runtime.
    * @param isRequiredVariable the {@code YoVariable}s do not get attached to this registry when the
    *                           variable is not a required variable and
    *                           {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                           {@code true}.
    * @return the unique {@code FBQuaternion3D} matching the search criteria.
    */
   public FBQuaternion3D getOrCreateOrientationData(RigidBodyBasics endEffector,
                                                    int controllerIndex,
                                                    Type type,
                                                    BooleanProvider activeFlag,
                                                    boolean isRequiredVariable)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateEndEffectorDataPool(endEffector, controllerIndex);
      FBQuaternion3D orientationData = dataPool.getOrCreateOrientationData(type, isRequiredVariable);
      orientationData.addActiveFlag(activeFlag);
      return orientationData;
   }

   /**
    * Retrieves and returns the {@code FBVector3D} associated with the given end-effector,
    * {@code type}, and {@code space}, if it does not exist it is created.
    * <p>
    * The name prefix of the created variable is created as follows:
    * 
    * <pre>
    * namePrefix = endEffector.getName() + type.getName() + space.getName()
    * </pre>
    * 
    * Such that the desired linear velocity for the rigid-body 'rightHand' will have the prefix:
    * "rightHandDesiredLinearVelocity".
    * </p>
    *
    * @param endEffector        the end-effector to which the returned data is associated.
    * @param controllerIndex    the index of the feedback controller requesting the data.
    * @param type               the type of the data to retrieve.
    * @param space              the space of the data to retrieve.
    * @param activeFlag         boolean provider that should reflect the active state of the controller
    *                           calling this factory. Used to determine whether the returned data is
    *                           up-to-date or not at runtime.
    * @param isRequiredVariable the {@code YoVariable}s do not get attached to this registry when the
    *                           variable is not a required variable and
    *                           {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                           {@code true}.
    * @return the unique {@code FBVector3D} matching the search criteria.
    */
   public FBVector3D getOrCreateVectorData3D(RigidBodyBasics endEffector,
                                             int controllerIndex,
                                             Type type,
                                             SpaceData3D space,
                                             BooleanProvider activeFlag,
                                             boolean isRequiredVariable)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateEndEffectorDataPool(endEffector, controllerIndex);
      FBVector3D vectorData = dataPool.getOrCreateVectorData3D(type, space, isRequiredVariable);
      vectorData.addActiveFlag(activeFlag);
      return vectorData;
   }

   /**
    * Retrieves and returns the {@code FBRateLimitedVector3D} associated with the given end-effector,
    * {@code type}, and {@code space}, if it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt} and {@code maximumRate} are only used if the data does not exist
    * yet.
    * </p>
    * <p>
    * The name prefix of the created variable is created as follows:
    * 
    * <pre>
    * namePrefix = endEffector.getName() + "RateLimited" + type.getName() + space.getName()
    * </pre>
    * 
    * Such that the rate-limited vector of the desired linear acceleration for the rigid-body
    * 'rightHand' will have the prefix: "rightHandRateLimitedDesiredLinearAcceleration".
    * </p>
    *
    * @param endEffector        the end-effector to which the returned data is associated.
    * @param controllerIndex    the index of the feedback controller requesting the data.
    * @param rawDataType        the type of the raw vector onto which the rate limit is to be applied.
    * @param space              the space of the data to retrieve.
    * @param dt                 the duration of a control tick.
    * @param maximumRate        the maximum rate allowed rate. Not modified.
    * @param activeFlag         boolean provider that should reflect the active state of the controller
    *                           calling this factory. Used to determine whether the returned data is
    *                           up-to-date or not at runtime.
    * @param isRequiredVariable the {@code YoVariable}s do not get attached to this registry when the
    *                           variable is not a required variable and
    *                           {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                           {@code true}.
    * @return the unique {@code FBRateLimitedVector3D} matching the search criteria.
    */
   public FBRateLimitedVector3D getOrCreateRateLimitedVectorData3D(RigidBodyBasics endEffector,
                                                                   int controllerIndex,
                                                                   Type rawDataType,
                                                                   SpaceData3D space,
                                                                   double dt,
                                                                   YoDouble maximumRate,
                                                                   BooleanProvider activeFlag,
                                                                   boolean isRequiredVariable)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateEndEffectorDataPool(endEffector, controllerIndex);
      FBRateLimitedVector3D rateLimitedVectorData = dataPool.getOrCreateRateLimitedVectorData(rawDataType, space, maximumRate, dt, isRequiredVariable);
      rateLimitedVectorData.addActiveFlag(activeFlag);
      return rateLimitedVectorData;
   }

   /**
    * Retrieves and returns the {@code FBAlphaFilteredVector3D} associated with the given end-effector,
    * {@code type}, and {@code space}, if it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt} and {@code breakFrequencyProvider} are only used if the data does
    * not exist yet.
    * </p>
    * <p>
    * The name prefix of the created variable is created as follows:
    * 
    * <pre>
    * namePrefix = endEffector.getName() + "Filtered" + type.getName() + space.getName()
    * </pre>
    * 
    * Such that the filtered vector of the linear velocity error for the rigid-body 'rightHand' will
    * have the prefix: "rightHandFilteredErrorLinearVelocity".
    * </p>
    *
    * @param endEffector            the end-effector to which the returned data is associated.
    * @param controllerIndex        the index of the feedback controller requesting the data.
    * @param rawDataType            the type of the raw vector onto which the rate limit is to be
    *                               applied.
    * @param space                  the space of the data to retrieve.
    * @param dt                     the duration of a control tick.
    * @param breakFrequencyProvider the break frequency to use for the low-pass filter. Not modified.
    * @param activeFlag             boolean provider that should reflect the active state of the
    *                               controller calling this factory. Used to determine whether the
    *                               returned data is up-to-date or not at runtime.
    * @param isRequiredVariable     the {@code YoVariable}s do not get attached to this registry when
    *                               the variable is not a required variable and
    *                               {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                               {@code true}.
    * @return the unique {@code FBAlphaFilteredVector3D} matching the search criteria.
    */
   public FBAlphaFilteredVector3D getOrCreateAlphaFilteredVectorData3D(RigidBodyBasics endEffector,
                                                                       int controllerIndex,
                                                                       Type rawDataType,
                                                                       SpaceData3D space,
                                                                       double dt,
                                                                       DoubleProvider breakFrequencyProvider,
                                                                       BooleanProvider activeFlag,
                                                                       boolean isRequiredVariable)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateEndEffectorDataPool(endEffector, controllerIndex);
      FBAlphaFilteredVector3D alphaFilteredVectorData = dataPool.getOrCreateAlphaFilteredVectorData(rawDataType,
                                                                                                    space,
                                                                                                    breakFrequencyProvider,
                                                                                                    dt,
                                                                                                    isRequiredVariable);
      alphaFilteredVectorData.addActiveFlag(activeFlag);
      return alphaFilteredVectorData;
   }

   /**
    * Retrieves and returns the {@code FBPose3D} associated with the given end-effector and
    * {@code type}, if it does not exist it is created.
    *
    * @param endEffector        the end-effector to which the returned data is associated.
    * @param controllerIndex    the index of the feedback controller requesting the data.
    * @param type               the type of the data to retrieve.
    * @param activeFlag         boolean provider that should reflect the active state of the controller
    *                           calling this factory. Used to determine whether the returned data is
    *                           up-to-date or not at runtime.
    * @param isRequiredVariable the {@code YoVariable}s do not get attached to this registry when the
    *                           variable is not a required variable and
    *                           {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                           {@code true}.
    * @return the unique {@code FBPose3D} matching the search criteria.
    */
   public FBPose3D getOrCreatePoseData(RigidBodyBasics endEffector, int controllerIndex, Type type, BooleanProvider activeFlag, boolean isRequiredVariable)
   {
      return new FBPose3D(getOrCreatePositionData(endEffector, controllerIndex, type, activeFlag, isRequiredVariable),
                          getOrCreateOrientationData(endEffector, controllerIndex, type, activeFlag, isRequiredVariable));
   }

   /**
    * Retrieves and returns the {@code FBVector6D} associated with the given end-effector,
    * {@code type}, and {@code space}, if it does not exist it is created.
    *
    * @param endEffector        the end-effector to which the returned data is associated.
    * @param controllerIndex    the index of the feedback controller requesting the data.
    * @param type               the type of the data to retrieve.
    * @param space              the space of the data to retrieve.
    * @param activeFlag         boolean provider that should reflect the active state of the controller
    *                           calling this factory. Used to determine whether the returned data is
    *                           up-to-date or not at runtime.
    * @param isRequiredVariable the {@code YoVariable}s do not get attached to this registry when the
    *                           variable is not a required variable and
    *                           {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                           {@code true}.
    * @return the unique {@code FBVector6D} matching the search criteria.
    */
   public FBVector6D getOrCreateVectorData6D(RigidBodyBasics endEffector,
                                             int controllerIndex,
                                             Type type,
                                             SpaceData6D space,
                                             BooleanProvider activeFlag,
                                             boolean isRequiredVariable)
   {
      SpaceData3D angularSpace = space == SpaceData6D.POSE ? SpaceData3D.ROTATION_VECTOR : space.getAngular();
      SpaceData3D linearSpace = space.getLinear();
      return new FBVector6D(getOrCreateVectorData3D(endEffector, controllerIndex, type, angularSpace, activeFlag, isRequiredVariable),
                            getOrCreateVectorData3D(endEffector, controllerIndex, type, linearSpace, activeFlag, isRequiredVariable));
   }

   /**
    * Retrieves and returns the {@code FBAlphaFilteredVector6D} for the filtered angular and linear
    * velocity errors of the given end-effector. If it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt}, {@code breakFrequencyLinearPart}, and
    * {@code breakFrequencyAngularPart} are only used if the data does not exist yet.
    * </p>
    *
    * @param endEffector               the end-effector to which the returned data is associated.
    * @param controllerIndex           the index of the feedback controller requesting the data.
    * @param rawDataType               the type of the raw vector onto which the filter is to be
    *                                  applied.
    * @param space                     the space of the data to retrieve.
    * @param dt                        the duration of a control tick.
    * @param breakFrequencyAngularPart the break frequency to use for the angular part of the velocity
    *                                  error. Not modified.
    * @param breakFrequencyLinearPart  the break frequency to use for the linear part of the velocity
    *                                  error. Not modified.
    * @param activeFlag                boolean provider that should reflect the active state of the
    *                                  controller calling this factory. Used to determine whether the
    *                                  returned data is up-to-date or not at runtime.
    * @param isRequiredVariable        the {@code YoVariable}s do not get attached to this registry
    *                                  when the variable is not a required variable and
    *                                  {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                                  {@code true}.
    * @return the unique {@code FBAlphaFilteredVector6D} matching the search criteria.
    */
   public FBAlphaFilteredVector6D getOrCreateAlphaFilteredVectorData6D(RigidBodyBasics endEffector,
                                                                       int controllerIndex,
                                                                       Type rawDataType,
                                                                       SpaceData6D space,
                                                                       double dt,
                                                                       DoubleProvider breakFrequencyAngularPart,
                                                                       DoubleProvider breakFrequencyLinearPart,
                                                                       BooleanProvider activeFlag,
                                                                       boolean isRequiredVariable)
   {
      SpaceData3D angularSpace = space == SpaceData6D.POSE ? SpaceData3D.ROTATION_VECTOR : space.getAngular();
      SpaceData3D linearSpace = space.getLinear();
      return new FBAlphaFilteredVector6D(getOrCreateAlphaFilteredVectorData3D(endEffector,
                                                                              controllerIndex,
                                                                              rawDataType,
                                                                              angularSpace,
                                                                              dt,
                                                                              breakFrequencyAngularPart,
                                                                              activeFlag,
                                                                              isRequiredVariable),
                                         getOrCreateAlphaFilteredVectorData3D(endEffector,
                                                                              controllerIndex,
                                                                              rawDataType,
                                                                              linearSpace,
                                                                              dt,
                                                                              breakFrequencyLinearPart,
                                                                              activeFlag,
                                                                              isRequiredVariable));
   }

   /**
    * Retrieves and returns the {@code FBRateLimitedVector6D} for the rate-limited angular and linear
    * accelerations of the given end-effector. The data type of the vector is defined by {@code type}.
    * If it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt}, {@code maximumLinearRate}, and {@code maximumAngularRate} are
    * only used if the data does not exist yet.
    * </p>
    *
    * @param endEffector        the end-effector to which the returned data is associated.
    * @param controllerIndex    the index of the feedback controller requesting the data.
    * @param rawDataType        the type of the raw vector onto which the rate limit is to be applied.
    * @param space              the space of the data to retrieve.
    * @param dt                 the duration of a control tick.
    * @param maximumAngularRate the maximum angular rate allowed rate. Not modified.
    * @param maximumLinearRate  the maximum linear rate allowed rate. Not modified.
    * @param activeFlag         boolean provider that should reflect the active state of the controller
    *                           calling this factory. Used to determine whether the returned data is
    *                           up-to-date or not at runtime.
    * @param isRequiredVariable the {@code YoVariable}s do not get attached to this registry when the
    *                           variable is not a required variable and
    *                           {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                           {@code true}.
    * @return the unique {@code FBRateLimitedVector6D} matching the search criteria.
    */
   public FBRateLimitedVector6D getOrCreateRateLimitedVectorData6D(RigidBodyBasics endEffector,
                                                                   int controllerIndex,
                                                                   Type rawDataType,
                                                                   SpaceData6D space,
                                                                   double dt,
                                                                   YoDouble maximumAngularRate,
                                                                   YoDouble maximumLinearRate,
                                                                   BooleanProvider activeFlag,
                                                                   boolean isRequiredVariable)
   {
      SpaceData3D angularSpace = space == SpaceData6D.POSE ? SpaceData3D.ROTATION_VECTOR : space.getAngular();
      SpaceData3D linearSpace = space.getLinear();
      return new FBRateLimitedVector6D(getOrCreateRateLimitedVectorData3D(endEffector,
                                                                          controllerIndex,
                                                                          rawDataType,
                                                                          angularSpace,
                                                                          dt,
                                                                          maximumAngularRate,
                                                                          activeFlag,
                                                                          isRequiredVariable),
                                       getOrCreateRateLimitedVectorData3D(endEffector,
                                                                          controllerIndex,
                                                                          rawDataType,
                                                                          linearSpace,
                                                                          dt,
                                                                          maximumLinearRate,
                                                                          activeFlag,
                                                                          isRequiredVariable));
   }

   /**
    * Retrieves and returns the set of orientation gains {@code YoPID3DGains} associated to the given
    * end-effector, if it does not exist it is created.
    *
    * @param endEffector        the end-effector to which the gains are associated.
    * @param controllerIndex    the index of the feedback controller requesting the data.
    * @param useIntegrator      whether to create the gains necessary to compute the integral term.
    * @param isRequiredVariable the {@code YoVariable}s do not get attached to this registry when the
    *                           variable is not a required variable and
    *                           {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                           {@code true}.
    * @return the unique {@code YoPID3DGains} associated with the given end-effector.
    */
   public YoPID3DGains getOrCreateOrientationGains(RigidBodyBasics endEffector, int controllerIndex, boolean useIntegrator, boolean isRequiredVariable)
   {
      return getOrCreateEndEffectorDataPool(endEffector, controllerIndex).getOrCreateOrientationGains(useIntegrator, isRequiredVariable);
   }

   /**
    * Retrieves and returns the set of position gains {@code YoPID3DGains} associated to the given
    * end-effector, if it does not exist it is created.
    *
    * @param endEffector        the end-effector to which the gains are associated.
    * @param controllerIndex    the index of the feedback controller requesting the data.
    * @param useIntegrator      whether to create the gains necessary to compute the integral term.
    * @param isRequiredVariable the {@code YoVariable}s do not get attached to this registry when the
    *                           variable is not a required variable and
    *                           {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                           {@code true}.
    * @return the unique {@code YoPID3DGains} associated with the given end-effector.
    */
   public YoPID3DGains getOrCreatePositionGains(RigidBodyBasics endEffector, int controllerIndex, boolean useIntegrator, boolean isRequiredVariable)
   {
      return getOrCreateEndEffectorDataPool(endEffector, controllerIndex).getOrCreatePositionGains(useIntegrator, isRequiredVariable);
   }

   /**
    * Retrieves and returns the set of gains {@code YoPIDSE3Gains} associated to the given
    * end-effector, if it does not exist it is created.
    *
    * @param endEffector        the end-effector to which the gains are associated.
    * @param controllerIndex    the index of the feedback controller requesting the data.
    * @param useIntegrator      whether to create the gains necessary to compute the integral term.
    * @param isRequiredVariable the {@code YoVariable}s do not get attached to this registry when the
    *                           variable is not a required variable and
    *                           {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                           {@code true}.
    * @return the unique {@code YoPIDSE3Gains} associated with the given end-effector.
    */
   public YoPIDSE3Gains getOrCreateSE3PIDGains(RigidBodyBasics endEffector, int controllerIndex, boolean useIntegrator, boolean isRequiredVariable)
   {
      YoPID3DGains positionGains = getOrCreatePositionGains(endEffector, controllerIndex, useIntegrator, isRequiredVariable);
      YoPID3DGains orientationGains = getOrCreateOrientationGains(endEffector, controllerIndex, useIntegrator, isRequiredVariable);
      return new DefaultYoPIDSE3Gains(positionGains, orientationGains);
   }

   /**
    * Retrieves and returns a spatial feedback control frame {@code YoSE3OffsetFrame} associated to the given
    * end-effector, if it does not exist it is created.
    *
    * @param endEffector        the end-effector to which the control frame is associated.
    * @param controllerIndex    the index of the feedback controller requesting the data.
    * @param isRequiredVariable the {@code YoVariable}s do not get attached to this registry when the
    *                           variable is not a required variable and
    *                           {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                           {@code true}.
    * @return a unique {@code YoSE3OffsetFrame} spatial feedback control frame associated with the given end-effector.
    */
   public YoSE3OffsetFrame getOrCreateSpatialFeedbackControlFrame(RigidBodyBasics endEffector, int controllerIndex, boolean isRequiredVariable)
   {
      return getOrCreateEndEffectorDataPool(endEffector, controllerIndex).getOrCreateSpatialFeedbackControlFrame(endEffector.getBodyFixedFrame(), isRequiredVariable);
   }

   /**
    * Retrieves and returns a point feedback control frame {@code YoTranslationFrame} associated to the given
    * end-effector, if it does not exist it is created.
    *
    * @param endEffector        the end-effector to which the control frame is associated.
    * @param controllerIndex    the index of the feedback controller requesting the data.
    * @param isRequiredVariable the {@code YoVariable}s do not get attached to this registry when the
    *                           variable is not a required variable and
    *                           {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                           {@code true}.
    * @return a unique {@code YoTranslationFrame} point feedback control frame associated with the given end-effector.
    */
   public YoTranslationFrame getOrCreatePointFeedbackControlFrame(RigidBodyBasics endEffector, int controllerIndex, boolean isRequiredVariable)
   {
      return getOrCreateEndEffectorDataPool(endEffector, controllerIndex).getOrCreatePointFeedbackControlFrame(endEffector.getBodyFixedFrame(), isRequiredVariable);
   }

   /**
    * Retrieves and returns an orientation feedback control frame {@code YoOrientationFrame} associated to the given
    * end-effector, if it does not exist it is created.
    *
    * @param endEffector        the end-effector to which the control frame is associated.
    * @param controllerIndex    the index of the feedback controller requesting the data.
    * @param isRequiredVariable the {@code YoVariable}s do not get attached to this registry when the
    *                           variable is not a required variable and
    *                           {@link WholeBodyControllerCore#REDUCE_YOVARIABLES} is set to
    *                           {@code true}.
    * @return a unique {@code YoOrientationFrame} orientation feedback control frame associated with the given end-effector.
    */
   public YoOrientationFrame getOrCreateOrientationFeedbackControlFrame(RigidBodyBasics endEffector, int controllerIndex, boolean isRequiredVariable)
   {
      return getOrCreateEndEffectorDataPool(endEffector, controllerIndex).getOrCreateOrientationFeedbackControlFrame(endEffector.getBodyFixedFrame(), isRequiredVariable);
   }

   /**
    * Calls {@link FeedbackControllerData#clearIfInactive()} to all the register objects used by the
    * feedback controllers.
    * <p>
    * The method should be called at the beginning of the controller core tick such that the unused
    * part of the data will be {@link Double#NaN} making it clear what it is used and what is not.
    * </p>
    */
   public void clearUnusedData()
   {
      for (int i = 0; i < singleFeedbackControllerDataPoolList.size(); i++)
      {
         singleFeedbackControllerDataPoolList.get(i).clearIfInactive();
      }
   }

   public DoubleProvider getErrorVelocityFilterBreakFrequency(String endEffectorOrJointName)
   {
      return errorVelocityFilterBreakFrequencies.get(endEffectorOrJointName);
   }

   @Override
   public void getCenterOfMassPositionData(List<FBPoint3D> positionDataListToPack, Type type)
   {
      positionDataListToPack.clear();
      FBPoint3D positionData = getOrCreateCenterOfMassDataPool().positionDataMap.get(type);

      if (positionData == null || !positionData.isActive())
         return;

      positionDataListToPack.add(positionData);
   }

   @Override
   public void getCenterOfMassVectorData(List<FBVector3D> vectorDataListToPack, Type type, SpaceData3D space)
   {
      vectorDataListToPack.clear();
      EnumMap<SpaceData3D, FBVector3D> endEffectorDataTyped = getOrCreateCenterOfMassDataPool().vectorDataMap.get(type);

      if (endEffectorDataTyped == null)
         return;

      FBVector3D vectorData = endEffectorDataTyped.get(space);

      if (vectorData == null || !vectorData.isActive())
         return;

      vectorDataListToPack.add(vectorData);
   }

   @Override
   public void getPositionData(RigidBodyBasics endEffector, List<FBPoint3D> positionDataListToPack, Type type)
   {
      positionDataListToPack.clear();
      List<SingleFeedbackControllerDataPool> dataPoolList = endEffectorDataPoolMap.get(endEffector);

      if (dataPoolList == null)
         return;

      for (int i = 0; i < dataPoolList.size(); i++)
      {
         EnumMap<Type, FBPoint3D> positionDataMap = dataPoolList.get(i).positionDataMap;
         FBPoint3D positionData = positionDataMap.get(type);
         if (positionData == null || !positionData.isActive())
            continue;

         positionDataListToPack.add(positionData);
      }
   }

   @Override
   public void getOrientationData(RigidBodyBasics endEffector, List<FBQuaternion3D> orientationDataListToPack, Type type)
   {
      orientationDataListToPack.clear();
      List<SingleFeedbackControllerDataPool> dataPoolList = endEffectorDataPoolMap.get(endEffector);

      if (dataPoolList == null)
         return;

      for (int i = 0; i < dataPoolList.size(); i++)
      {
         EnumMap<Type, FBQuaternion3D> orientationDataMap = dataPoolList.get(i).orientationDataMap;
         FBQuaternion3D orientationData = orientationDataMap.get(type);
         if (orientationData == null || !orientationData.isActive())
            continue;

         orientationDataListToPack.add(orientationData);
      }
   }

   @Override
   public void getVectorData(RigidBodyBasics endEffector, List<FBVector3D> vectorDataListToPack, Type type, SpaceData3D space)
   {
      vectorDataListToPack.clear();
      List<SingleFeedbackControllerDataPool> dataPoolList = endEffectorDataPoolMap.get(endEffector);

      if (dataPoolList == null)
         return;

      for (int i = 0; i < dataPoolList.size(); i++)
      {
         EnumMap<Type, EnumMap<SpaceData3D, FBVector3D>> vectorDataMap = dataPoolList.get(i).vectorDataMap;
         EnumMap<SpaceData3D, FBVector3D> vectorDataSubMap = vectorDataMap.get(type);
         if (vectorDataSubMap == null)
            continue;

         FBVector3D vectorData = vectorDataSubMap.get(space);

         if (vectorData == null || !vectorData.isActive())
            continue;

         vectorDataListToPack.add(vectorData);
      }
   }

   @Override
   public InverseDynamicsCommandList getLastFeedbackControllerInverseDynamicsOutput()
   {
      return lastFeedbackControllerInverseDynamicsOutput;
   }

   @Override
   public InverseKinematicsCommandList getLastFeedbackControllerInverseKinematicsOutput()
   {
      return lastFeedbackControllerInverseKinematicsOutput;
   }

   @Override
   public VirtualModelControlCommandList getLastFeedbackControllerVirtualModelControlOutput()
   {
      return lastFeedbackControllerVirtualModelControlOutput;
   }

   public static String appendIndex(String input, int index)
   {
      if (index == 0)
         return input;
      else
         return input + Integer.toString(index);
   }

   private static class SingleFeedbackControllerDataPool
   {
      private final YoRegistry registry;
      private final YoRegistry debugRegistry;
      private final String namePrefix;
      private final String controlFrameNameSuffix = "BodyFixedControlFrame";

      private final EnumMap<Type, FBPoint3D> positionDataMap = new EnumMap<>(Type.class);
      private final EnumMap<Type, FBQuaternion3D> orientationDataMap = new EnumMap<>(Type.class);
      private final EnumMap<Type, EnumMap<SpaceData3D, FBVector3D>> vectorDataMap = new EnumMap<>(Type.class);
      private final EnumMap<Type, EnumMap<SpaceData3D, FBRateLimitedVector3D>> rateLimitedVectorDataMap = new EnumMap<>(Type.class);
      private final EnumMap<Type, EnumMap<SpaceData3D, FBAlphaFilteredVector3D>> filteredVectorDataMap = new EnumMap<>(Type.class);

      private YoPID3DGains orientationGains;
      private YoPID3DGains positionGains;

      private YoFrameVector3D controlFrameTranslationToParent;
      private YoFrameQuaternion controlFrameRotationToParent;

      private YoSE3OffsetFrame spatialFeedbackControlFrame;
      private YoTranslationFrame pointFeedbackControlFrame;
      private YoOrientationFrame orientationFeedbackControlFrame;

      private final List<FeedbackControllerData> clearableData = new ArrayList<>();

      public SingleFeedbackControllerDataPool(String namePrefix, int controllerIndex, YoRegistry registry)
      {
         this.namePrefix = appendIndex(namePrefix, controllerIndex);
         this.registry = registry;
         debugRegistry = WholeBodyControllerCore.REDUCE_YOVARIABLES ? null : registry;
      }

      public void clearIfInactive()
      {
         for (int i = 0; i < clearableData.size(); i++)
         {
            clearableData.get(i).clearIfInactive();
         }
      }

      public FBPoint3D getOrCreatePositionData(Type type, boolean isRequiredVariable)
      {
         FBPoint3D positionData = positionDataMap.get(type);

         if (positionData == null)
         {
            positionData = new FBPoint3D(namePrefix, type, isRequiredVariable ? registry : debugRegistry);
            positionDataMap.put(type, positionData);
            clearableData.add(positionData);
         }

         return positionData;
      }

      public FBQuaternion3D getOrCreateOrientationData(Type type, boolean isRequiredVariable)
      {
         FBQuaternion3D orientationData = orientationDataMap.get(type);

         if (orientationData == null)
         {
            orientationData = new FBQuaternion3D(namePrefix, type, isRequiredVariable ? registry : debugRegistry);
            orientationDataMap.put(type, orientationData);
            clearableData.add(orientationData);
         }

         return orientationData;
      }

      public FBVector3D getOrCreateVectorData3D(Type type, SpaceData3D space, boolean isRequiredVariable)
      {
         EnumMap<SpaceData3D, FBVector3D> vectorDataSubMap = getSubEnumMap(vectorDataMap, type, SpaceData3D.class);
         FBVector3D vectorData = vectorDataSubMap.get(space);

         if (vectorData == null)
         {
            vectorData = new FBVector3D(namePrefix, type, space, isRequiredVariable ? registry : debugRegistry);
            vectorDataSubMap.put(space, vectorData);
            clearableData.add(vectorData);
         }

         return vectorData;
      }

      public FBAlphaFilteredVector3D getOrCreateAlphaFilteredVectorData(Type type,
                                                                        SpaceData3D space,
                                                                        DoubleProvider breakFrequency,
                                                                        double dt,
                                                                        boolean isRequiredVariable)
      {
         EnumMap<SpaceData3D, FBAlphaFilteredVector3D> filteredVectorDataSubMap = getSubEnumMap(filteredVectorDataMap, type, SpaceData3D.class);
         FBAlphaFilteredVector3D filteredVectorData = filteredVectorDataSubMap.get(space);

         if (filteredVectorData == null)
         {
            FBVector3D rawVectorData = getOrCreateVectorData3D(type, space, isRequiredVariable);
            filteredVectorData = new FBAlphaFilteredVector3D(namePrefix,
                                                             type,
                                                             space,
                                                             breakFrequency,
                                                             dt,
                                                             rawVectorData,
                                                             isRequiredVariable ? registry : debugRegistry);
            filteredVectorDataSubMap.put(space, filteredVectorData);
            clearableData.add(filteredVectorData);
         }

         return filteredVectorData;
      }

      public FBRateLimitedVector3D getOrCreateRateLimitedVectorData(Type type,
                                                                    SpaceData3D space,
                                                                    DoubleProvider maximumRate,
                                                                    double dt,
                                                                    boolean isRequiredVariable)
      {
         EnumMap<SpaceData3D, FBRateLimitedVector3D> rateLimitedVectorDataSubMap = getSubEnumMap(rateLimitedVectorDataMap, type, SpaceData3D.class);
         FBRateLimitedVector3D rateLimitedVectorData = rateLimitedVectorDataSubMap.get(space);

         if (rateLimitedVectorData == null)
         {
            FBVector3D rawVectorData = getOrCreateVectorData3D(type, space, isRequiredVariable);
            rateLimitedVectorData = new FBRateLimitedVector3D(namePrefix,
                                                              type,
                                                              space,
                                                              maximumRate,
                                                              dt,
                                                              rawVectorData,
                                                              isRequiredVariable ? registry : debugRegistry);
            rateLimitedVectorDataSubMap.put(space, rateLimitedVectorData);
            clearableData.add(rateLimitedVectorData);
         }

         return rateLimitedVectorData;
      }

      public YoPID3DGains getOrCreateOrientationGains(boolean useIntegrator, boolean isRequiredVariable)
      {
         if (orientationGains == null)
            orientationGains = new DefaultYoPID3DGains(namePrefix + "Orientation",
                                                       GainCoupling.NONE,
                                                       useIntegrator,
                                                       isRequiredVariable ? registry : debugRegistry);
         return orientationGains;
      }

      public YoPID3DGains getOrCreatePositionGains(boolean useIntegrator, boolean isRequiredVariable)
      {
         if (positionGains == null)
            positionGains = new DefaultYoPID3DGains(namePrefix + "Position", GainCoupling.NONE, useIntegrator, isRequiredVariable ? registry : debugRegistry);
         return positionGains;
      }

      public YoSE3OffsetFrame getOrCreateSpatialFeedbackControlFrame(ReferenceFrame parentFrame, boolean isRequiredVariable)
      {
         if (spatialFeedbackControlFrame == null)
            spatialFeedbackControlFrame = new YoSE3OffsetFrame(namePrefix + "Spatial" + controlFrameNameSuffix,
                                                               getOrCreateControlFrameTranslationOffset(parentFrame, isRequiredVariable),
                                                               getOrCreateControlFrameOrientationOffset(parentFrame, isRequiredVariable),
                                                               parentFrame);
         return spatialFeedbackControlFrame;
      }

      public YoTranslationFrame getOrCreatePointFeedbackControlFrame(ReferenceFrame parentFrame, boolean isRequiredVariable)
      {
         if (pointFeedbackControlFrame == null)
            pointFeedbackControlFrame = new YoTranslationFrame(namePrefix + "Point" + controlFrameNameSuffix,
                                                               getOrCreateControlFrameTranslationOffset(parentFrame, isRequiredVariable),
                                                               parentFrame);
         return pointFeedbackControlFrame;
      }

      public YoOrientationFrame getOrCreateOrientationFeedbackControlFrame(ReferenceFrame parentFrame, boolean isRequiredVariable)
      {
         if (orientationFeedbackControlFrame == null)
            orientationFeedbackControlFrame = new YoOrientationFrame(namePrefix + "Orientation" + controlFrameNameSuffix,
                                                                     getOrCreateControlFrameOrientationOffset(parentFrame, isRequiredVariable),
                                                                     parentFrame);
         return orientationFeedbackControlFrame;
      }

      private YoFrameVector3D getOrCreateControlFrameTranslationOffset(ReferenceFrame parentFrame, boolean isRequiredVariable)
      {
         if (controlFrameTranslationToParent == null)
            controlFrameTranslationToParent = new YoFrameVector3D(namePrefix + controlFrameNameSuffix, parentFrame, isRequiredVariable ? registry : debugRegistry);
         return controlFrameTranslationToParent;
      }

      private YoFrameQuaternion getOrCreateControlFrameOrientationOffset(ReferenceFrame parentFrame, boolean isRequiredVariable)
      {
         if (controlFrameRotationToParent == null)
            controlFrameRotationToParent = new YoFrameQuaternion(namePrefix + controlFrameNameSuffix, parentFrame, isRequiredVariable ? registry : debugRegistry);
         return controlFrameRotationToParent;
      }

      private static <K, E extends Enum<E>, V> EnumMap<E, V> getSubEnumMap(Map<K, EnumMap<E, V>> enclosingMap, K key, Class<E> subMapEnumType)
      {
         EnumMap<E, V> subMap = enclosingMap.get(key);
         if (subMap == null)
         {
            subMap = new EnumMap<>(subMapEnumType);
            enclosingMap.put(key, subMap);
         }
         return subMap;
      }
   }
}
