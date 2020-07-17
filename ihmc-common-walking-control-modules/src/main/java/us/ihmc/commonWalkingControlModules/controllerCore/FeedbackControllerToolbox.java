package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.controlModules.YoSE3OffsetFrame;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.data.AlphaFilteredVectorData3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.AlphaFilteredVectorData6D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FeedbackControllerData;
import us.ihmc.commonWalkingControlModules.controllerCore.data.PoseData3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.PositionData3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.QuaternionData3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.RateLimitedVectorData3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.RateLimitedVectorData6D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.SpaceData3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.SpaceData6D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.data.VectorData3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.VectorData6D;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
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

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private SingleFeedbackControllerDataPool centerOfMassDataPool;
   private final Map<RigidBodyBasics, List<SingleFeedbackControllerDataPool>> endEffectorDataPoolMap = new HashMap<>();
   private final List<SingleFeedbackControllerDataPool> singleFeedbackControllerDataPoolList = new ArrayList<>();

   private final Map<String, DoubleProvider> errorVelocityFilterBreakFrequencies;

   private final InverseDynamicsCommandList lastFeedbackControllerInverseDynamicsOutput = new InverseDynamicsCommandList();
   private final InverseKinematicsCommandList lastFeedbackControllerInverseKinematicsOutput = new InverseKinematicsCommandList();
   private final VirtualModelControlCommandList lastFeedbackControllerVirtualModelControlOutput = new VirtualModelControlCommandList();

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

   public void registerFeedbackControllerOutput(InverseDynamicsCommandList output)
   {
      lastFeedbackControllerInverseDynamicsOutput.set(output);
      lastFeedbackControllerInverseKinematicsOutput.clear();
      lastFeedbackControllerVirtualModelControlOutput.clear();
   }

   public void registerFeedbackControllerOutput(InverseKinematicsCommandList output)
   {
      lastFeedbackControllerInverseDynamicsOutput.clear();
      lastFeedbackControllerInverseKinematicsOutput.set(output);
      lastFeedbackControllerVirtualModelControlOutput.clear();
   }

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
    * Retrieves and returns the {@code PositionData3D} for the center of mass associated with the given
    * {@code type}, if it does not exist it is created.
    *
    * @param type the type of the data to retrieve.
    * @return the unique {@code PositionData3D} matching the search criterion.
    */
   public PositionData3D getOrCreateCenterOfMassPositionData(Type type, YoBoolean enabled)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateCenterOfMassDataPool();
      PositionData3D positionData = dataPool.getOrCreatePositionData(type);
      positionData.addActiveFlag(enabled);
      return positionData;
   }

   /**
    * Retrieves and returns the {@code VectorData3D} for the center of mass associated with the given
    * {@code type}, and {@code space}, if it does not exist it is created.
    *
    * @param type  the type of the data to retrieve.
    * @param space the space of the data to retrieve.
    * @return the unique {@code VectorData3D} matching the search criteria.
    */
   public VectorData3D getOrCreateCenterOfMassVectorData(Type type, SpaceData3D space, YoBoolean enabled)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateCenterOfMassDataPool();
      VectorData3D vectorData = dataPool.getOrCreateVectorData3D(type, space);
      vectorData.addActiveFlag(enabled);
      return vectorData;
   }

   /**
    * Retrieves and returns the {@code AlphaFilteredVectorData3D} for the center of mass associated
    * with the given {@code type} and {@code space}, if it does not exist it is created.
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
    * @return the unique {@code AlphaFilteredVectorData3D} matching the search criteria.
    */
   public AlphaFilteredVectorData3D getOrCreateCenterOfMassAlphaFilteredVectorData(Type rawDataType, SpaceData3D space, double dt,
                                                                                   DoubleProvider breakFrequencyProvider, YoBoolean enabled)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateCenterOfMassDataPool();
      AlphaFilteredVectorData3D filteredVectorData = dataPool.getOrCreateAlphaFilteredVectorData(rawDataType, space, breakFrequencyProvider, dt);
      filteredVectorData.addActiveFlag(enabled);
      return filteredVectorData;
   }

   /**
    * Retrieves and returns the {@code RateLimitedVectorData3D} for the center of mass associated with
    * the given {@code type}, and {@code space}, if it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt} and {@code maximumRate} are only used if the data does not exist
    * yet.
    * </p>
    *
    * @param space       the space of the data to retrieve.
    * @param rawDataType the type of the raw vector onto which the rate limit is to be applied.
    * @param dt          the duration of a control tick.
    * @param maximumRate the maximum rate allowed rate. Not modified.
    * @return the unique {@code RateLimitedVectorData3D} matching the search criteria.
    */
   public RateLimitedVectorData3D getOrCreateCenterOfMassRateLimitedVectorData(Type rawDataType, SpaceData3D space, double dt, YoDouble maximumRate,
                                                                               YoBoolean enabled)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateCenterOfMassDataPool();
      RateLimitedVectorData3D rateLimitedVectorData = dataPool.getOrCreateRateLimitedVectorData(rawDataType, space, maximumRate, dt);
      rateLimitedVectorData.addActiveFlag(enabled);
      return rateLimitedVectorData;
   }

   /**
    * Retrieves and returns the set of gains {@code YoPositionPIDGainsInterface} for the center of
    * mass, if it does not exist it is created.
    *
    * @param useIntegrator whether to create the gains necessary to compute the integral term.
    * @return the unique {@code YoPositionPIDGainsInterface} for the center of mass.
    */
   public YoPID3DGains getOrCreateCenterOfMassGains(boolean useIntegrator)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateCenterOfMassDataPool();
      return dataPool.getOrCreatePositionGains(useIntegrator);
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
    * Retrieves and returns the {@code PositionData3D} associated with the given end-effector and
    * {@code type}, if it does not exist it is created.
    * <p>
    * The name prefix of the created variable is created as follows:<br>
    * {@code namePrefix = endEffector.getName() + type.getName() +}
    * {@link SpaceData3D#POSITION}{@code .getName()}<br>
    * Such that the desired position for the rigid-body 'rightHand' will have the prefix:
    * "rightHandDesiredPosition".
    * </p>
    *
    * @param endEffector     the end-effector to which the returned data is associated.
    * @param controllerIndex TODO
    * @param type            the type of the data to retrieve.
    * @return the unique {@code PositionData3D} matching the search criteria.
    */
   public PositionData3D getOrCreatePositionData(RigidBodyBasics endEffector, int controllerIndex, Type type, YoBoolean enabled)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateEndEffectorDataPool(endEffector, controllerIndex);
      PositionData3D positionData = dataPool.getOrCreatePositionData(type);
      positionData.addActiveFlag(enabled);
      return positionData;
   }

   /**
    * Retrieves and returns the {@code QuaternionData3D} associated with the given end-effector and
    * {@code type}, if it does not exist it is created.
    * <p>
    * The name prefix of the created variable is created as follows:<br>
    * {@code namePrefix = endEffector.getName() + type.getName() +}
    * {@link SpaceData3D#ORIENTATION}{@code .getName()}<br>
    * Such that the current orientation for the rigid-body 'rightHand' will have the prefix:
    * "rightHandCurrentOrientation".
    * </p>
    *
    * @param endEffector     the end-effector to which the returned data is associated.
    * @param controllerIndex TODO
    * @param type            the type of the data to retrieve.
    * @return the unique {@code QuaternionData3D} matching the search criteria.
    */
   public QuaternionData3D getOrCreateOrientationData(RigidBodyBasics endEffector, int controllerIndex, Type type, YoBoolean enabled)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateEndEffectorDataPool(endEffector, controllerIndex);
      QuaternionData3D orientationData = dataPool.getOrCreateOrientationData(type);
      orientationData.addActiveFlag(enabled);
      return orientationData;
   }

   /**
    * Retrieves and returns the {@code VectorData3D} associated with the given end-effector,
    * {@code type}, and {@code space}, if it does not exist it is created.
    * <p>
    * The name prefix of the created variable is created as follows:<br>
    * {@code namePrefix = endEffector.getName() + type.getName() + space.getName()}<br>
    * Such that the desired linear velocity for the rigid-body 'rightHand' will have the prefix:
    * "rightHandDesiredLinearVelocity".
    * </p>
    *
    * @param endEffector     the end-effector to which the returned data is associated.
    * @param controllerIndex TODO
    * @param type            the type of the data to retrieve.
    * @param space           the space of the data to retrieve.
    * @return the unique {@code VectorData3D} matching the search criteria.
    */
   public VectorData3D getOrCreateVectorData3D(RigidBodyBasics endEffector, int controllerIndex, Type type, SpaceData3D space, YoBoolean enabled)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateEndEffectorDataPool(endEffector, controllerIndex);
      VectorData3D vectorData = dataPool.getOrCreateVectorData3D(type, space);
      vectorData.addActiveFlag(enabled);
      return vectorData;
   }

   /**
    * Retrieves and returns the {@code RateLimitedVectorData3D} associated with the given end-effector,
    * {@code type}, and {@code space}, if it does not exist it is created.
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
    * @param endEffector     the end-effector to which the returned data is associated.
    * @param controllerIndex TODO
    * @param rawDataType     the type of the raw vector onto which the rate limit is to be applied.
    * @param space           the space of the data to retrieve.
    * @param dt              the duration of a control tick.
    * @param maximumRate     the maximum rate allowed rate. Not modified.
    * @return the unique {@code RateLimitedVectorData3D} matching the search criteria.
    */
   public RateLimitedVectorData3D getOrCreateRateLimitedVectorData3D(RigidBodyBasics endEffector, int controllerIndex, Type rawDataType, SpaceData3D space,
                                                                     double dt, YoDouble maximumRate, YoBoolean enabled)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateEndEffectorDataPool(endEffector, controllerIndex);
      RateLimitedVectorData3D rateLimitedVectorData = dataPool.getOrCreateRateLimitedVectorData(rawDataType, space, maximumRate, dt);
      rateLimitedVectorData.addActiveFlag(enabled);
      return rateLimitedVectorData;
   }

   /**
    * Retrieves and returns the {@code AlphaFilteredVectorData3D} associated with the given
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
    * @param endEffector            the end-effector to which the returned data is associated.
    * @param controllerIndex        TODO
    * @param rawDataType            the type of the raw vector onto which the rate limit is to be
    *                               applied.
    * @param space                  the space of the data to retrieve.
    * @param dt                     the duration of a control tick.
    * @param breakFrequencyProvider the break frequency to use for the low-pass filter. Not modified.
    * @return the unique {@code AlphaFilteredVectorData3D} matching the search criteria.
    */
   public AlphaFilteredVectorData3D getOrCreateAlphaFilteredVectorData(RigidBodyBasics endEffector, int controllerIndex, Type rawDataType, SpaceData3D space,
                                                                       double dt, DoubleProvider breakFrequencyProvider, YoBoolean enabled)
   {
      SingleFeedbackControllerDataPool dataPool = getOrCreateEndEffectorDataPool(endEffector, controllerIndex);
      AlphaFilteredVectorData3D alphaFilteredVectorData = dataPool.getOrCreateAlphaFilteredVectorData(rawDataType, space, breakFrequencyProvider, dt);
      alphaFilteredVectorData.addActiveFlag(enabled);
      return alphaFilteredVectorData;
   }

   /**
    * Retrieves and returns the {@code PoseData3D} associated with the given end-effector and
    * {@code type}, if it does not exist it is created.
    *
    * @param endEffector     the end-effector to which the returned data is associated.
    * @param controllerIndex TODO
    * @param type            the type of the data to retrieve.
    * @return the unique {@code PoseData3D} matching the search criteria.
    */
   public PoseData3D getOrCreatePoseData(RigidBodyBasics endEffector, int controllerIndex, Type type, YoBoolean enabled)
   {
      return new PoseData3D(getOrCreatePositionData(endEffector, controllerIndex, type, enabled),
                            getOrCreateOrientationData(endEffector, controllerIndex, type, enabled));
   }

   /**
    * Retrieves and returns the {@code VectorData6D} associated with the given end-effector,
    * {@code type}, and {@code space}, if it does not exist it is created.
    *
    * @param endEffector     the end-effector to which the returned data is associated.
    * @param controllerIndex TODO
    * @param type            the type of the data to retrieve.
    * @return the unique {@code VectorData6D} matching the search criteria.
    */
   public VectorData6D getOrCreateVectorData6D(RigidBodyBasics endEffector, int controllerIndex, Type type, SpaceData6D space, YoBoolean enabled)
   {
      return new VectorData6D(getOrCreateVectorData3D(endEffector, controllerIndex, type, space.getAngular(), enabled),
                              getOrCreateVectorData3D(endEffector, controllerIndex, type, space.getLinear(), enabled));
   }

   /**
    * Retrieves and returns the {@code AlphaFilteredVectorData6D} for the filtered angular and linear
    * velocity errors of the given end-effector. If it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt}, {@code breakFrequencyLinearPart}, and
    * {@code breakFrequencyAngularPart} are only used if the data does not exist yet.
    * </p>
    *
    * @param endEffector               the end-effector to which the returned data is associated.
    * @param controllerIndex           TODO
    * @param rawDataType               the type of the raw vector onto which the filter is to be
    *                                  applied.
    * @param dt                        the duration of a control tick.
    * @param breakFrequencyAngularPart the break frequency to use for the angular part of the velocity
    *                                  error. Not modified.
    * @param breakFrequencyLinearPart  the break frequency to use for the linear part of the velocity
    *                                  error. Not modified.
    * @return the unique {@code AlphaFilteredVectorData6D} matching the search criteria.
    */
   public AlphaFilteredVectorData6D getOrCreateAlphaFilteredVectorData6D(RigidBodyBasics endEffector, int controllerIndex, Type rawDataType, SpaceData6D space,
                                                                         double dt, DoubleProvider breakFrequencyAngularPart,
                                                                         DoubleProvider breakFrequencyLinearPart, YoBoolean enabled)
   {
      return new AlphaFilteredVectorData6D(getOrCreateAlphaFilteredVectorData(endEffector,
                                                                              controllerIndex,
                                                                              rawDataType,
                                                                              space.getAngular(),
                                                                              dt,
                                                                              breakFrequencyAngularPart,
                                                                              enabled),
                                           getOrCreateAlphaFilteredVectorData(endEffector,
                                                                              controllerIndex,
                                                                              rawDataType,
                                                                              space.getLinear(),
                                                                              dt,
                                                                              breakFrequencyLinearPart,
                                                                              enabled));
   }

   /**
    * Retrieves and returns the {@code RateLimitedYoMutableSpatialVector} for the rate-limited angular
    * and linear accelerations of the given end-effector. The data type of the vector is defined by
    * {@code type}. If it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt}, {@code maximumLinearRate}, and {@code maximumAngularRate} are
    * only used if the data does not exist yet.
    * </p>
    *
    * @param endEffector        the end-effector to which the returned data is associated.
    * @param controllerIndex    TODO
    * @param rawDataType        the type of the raw vector onto which the rate limit is to be applied.
    * @param dt                 the duration of a control tick.
    * @param maximumAngularRate the maximum angular rate allowed rate. Not modified.
    * @param maximumLinearRate  the maximum linear rate allowed rate. Not modified.
    * @return the unique {@code RateLimitedYoMutableSpatialVector} matching the search criteria.
    */
   public RateLimitedVectorData6D getOrCreateRateLimitedVectorData6D(RigidBodyBasics endEffector, int controllerIndex, Type rawDataType, SpaceData6D space,
                                                                     double dt, YoDouble maximumAngularRate, YoDouble maximumLinearRate, YoBoolean enabled)
   {
      return new RateLimitedVectorData6D(getOrCreateRateLimitedVectorData3D(endEffector,
                                                                            controllerIndex,
                                                                            rawDataType,
                                                                            space.getAngular(),
                                                                            dt,
                                                                            maximumAngularRate,
                                                                            enabled),
                                         getOrCreateRateLimitedVectorData3D(endEffector,
                                                                            controllerIndex,
                                                                            rawDataType,
                                                                            space.getLinear(),
                                                                            dt,
                                                                            maximumLinearRate,
                                                                            enabled));
   }

   /**
    * Retrieves and returns the set of orientation gains {@code YoPID3DGains} associated to the given
    * end-effector, if it does not exist it is created.
    *
    * @param endEffector     the end-effector to which the gains are associated.
    * @param controllerIndex TODO
    * @param useIntegrator   whether to create the gains necessary to compute the integral term.
    * @return the unique {@code YoPID3DGains} associated with the given end-effector.
    */
   public YoPID3DGains getOrCreateOrientationGains(RigidBodyBasics endEffector, int controllerIndex, boolean useIntegrator)
   {
      return getOrCreateEndEffectorDataPool(endEffector, controllerIndex).getOrCreateOrientationGains(useIntegrator);
   }

   /**
    * Retrieves and returns the set of position gains {@code YoPID3DGains} associated to the given
    * end-effector, if it does not exist it is created.
    *
    * @param endEffector     the end-effector to which the gains are associated.
    * @param controllerIndex TODO
    * @param useIntegrator   whether to create the gains necessary to compute the integral term.
    * @return the unique {@code YoPID3DGains} associated with the given end-effector.
    */
   public YoPID3DGains getOrCreatePositionGains(RigidBodyBasics endEffector, int controllerIndex, boolean useIntegrator)
   {
      return getOrCreateEndEffectorDataPool(endEffector, controllerIndex).getOrCreatePositionGains(useIntegrator);
   }

   /**
    * Retrieves and returns the set of gains {@code YoPIDSE3Gains} associated to the given
    * end-effector, if it does not exist it is created.
    *
    * @param endEffector     the end-effector to which the gains are associated.
    * @param controllerIndex TODO
    * @param useIntegrator   whether to create the gains necessary to compute the integral term.
    * @return the unique {@code YoPIDSE3Gains} associated with the given end-effector.
    */
   public YoPIDSE3Gains getOrCreateSE3PIDGains(RigidBodyBasics endEffector, int controllerIndex, boolean useIntegrator)
   {
      YoPID3DGains positionGains = getOrCreatePositionGains(endEffector, controllerIndex, useIntegrator);
      YoPID3DGains orientationGains = getOrCreateOrientationGains(endEffector, controllerIndex, useIntegrator);
      return new DefaultYoPIDSE3Gains(positionGains, orientationGains);
   }

   /**
    * Retrieves and returns the control frame {@code YoSE3OffsetFrame} associated to the given
    * end-effector, if it does not exist it is created.
    *
    * @param endEffector     the end-effector to which the control frame is associated.
    * @param controllerIndex TODO
    * @return the unique {@code YoSE3OffsetFrame} control frame associated with the given end-effector.
    */
   public YoSE3OffsetFrame getOrCreateControlFrame(RigidBodyBasics endEffector, int controllerIndex)
   {
      return getOrCreateEndEffectorDataPool(endEffector, controllerIndex).getOrCreateControlFrame(endEffector.getBodyFixedFrame());
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
   public void getCenterOfMassPositionData(List<PositionData3D> positionDataListToPack, Type type)
   {
      positionDataListToPack.clear();
      PositionData3D positionData = getOrCreateCenterOfMassDataPool().positionDataMap.get(type);

      if (positionData == null || !positionData.isActive())
         return;

      positionDataListToPack.add(positionData);
   }

   @Override
   public void getCenterOfMassVectorData(List<VectorData3D> vectorDataListToPack, Type type, SpaceData3D space)
   {
      vectorDataListToPack.clear();
      EnumMap<SpaceData3D, VectorData3D> endEffectorDataTyped = getOrCreateCenterOfMassDataPool().vectorDataMap.get(type);

      if (endEffectorDataTyped == null)
         return;

      VectorData3D vectorData = endEffectorDataTyped.get(space);

      if (vectorData == null || !vectorData.isActive())
         return;

      vectorDataListToPack.add(vectorData);
   }

   @Override
   public void getPositionData(RigidBodyBasics endEffector, List<PositionData3D> positionDataListToPack, Type type)
   {
      positionDataListToPack.clear();
      List<SingleFeedbackControllerDataPool> dataPoolList = endEffectorDataPoolMap.get(endEffector);

      if (dataPoolList == null)
         return;

      for (int i = 0; i < dataPoolList.size(); i++)
      {
         EnumMap<Type, PositionData3D> positionDataMap = dataPoolList.get(i).positionDataMap;
         PositionData3D positionData = positionDataMap.get(type);
         if (positionData == null || !positionData.isActive())
            continue;

         positionDataListToPack.add(positionData);
      }
   }

   @Override
   public void getOrientationData(RigidBodyBasics endEffector, List<QuaternionData3D> orientationDataListToPack, Type type)
   {
      orientationDataListToPack.clear();
      List<SingleFeedbackControllerDataPool> dataPoolList = endEffectorDataPoolMap.get(endEffector);

      if (dataPoolList == null)
         return;

      for (int i = 0; i < dataPoolList.size(); i++)
      {
         EnumMap<Type, QuaternionData3D> orientationDataMap = dataPoolList.get(i).orientationDataMap;
         QuaternionData3D orientationData = orientationDataMap.get(type);
         if (orientationData == null || !orientationData.isActive())
            continue;

         orientationDataListToPack.add(orientationData);
      }
   }

   @Override
   public void getVectorData(RigidBodyBasics endEffector, List<VectorData3D> vectorDataListToPack, Type type, SpaceData3D space)
   {
      vectorDataListToPack.clear();
      List<SingleFeedbackControllerDataPool> dataPoolList = endEffectorDataPoolMap.get(endEffector);

      if (dataPoolList == null)
         return;

      for (int i = 0; i < dataPoolList.size(); i++)
      {
         EnumMap<Type, EnumMap<SpaceData3D, VectorData3D>> vectorDataMap = dataPoolList.get(i).vectorDataMap;
         EnumMap<SpaceData3D, VectorData3D> vectorDataSubMap = vectorDataMap.get(type);
         if (vectorDataSubMap == null)
            continue;

         VectorData3D vectorData = vectorDataSubMap.get(space);

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
      private final YoVariableRegistry registry;
      private final String namePrefix;
      private final EnumMap<Type, PositionData3D> positionDataMap = new EnumMap<>(Type.class);
      private final EnumMap<Type, QuaternionData3D> orientationDataMap = new EnumMap<>(Type.class);
      private final EnumMap<Type, EnumMap<SpaceData3D, VectorData3D>> vectorDataMap = new EnumMap<>(Type.class);
      private final EnumMap<Type, EnumMap<SpaceData3D, RateLimitedVectorData3D>> rateLimitedVectorDataMap = new EnumMap<>(Type.class);
      private final EnumMap<Type, EnumMap<SpaceData3D, AlphaFilteredVectorData3D>> filteredVectorDataMap = new EnumMap<>(Type.class);

      private YoPID3DGains orientationGains;
      private YoPID3DGains positionGains;
      private YoSE3OffsetFrame controlFrame;

      private final List<FeedbackControllerData> clearableData = new ArrayList<>();

      public SingleFeedbackControllerDataPool(String namePrefix, int controllerIndex, YoVariableRegistry registry)
      {
         this.namePrefix = appendIndex(namePrefix, controllerIndex);
         this.registry = registry;
      }

      public void clearIfInactive()
      {
         for (int i = 0; i < clearableData.size(); i++)
         {
            clearableData.get(i).clearIfInactive();
         }
      }

      public PositionData3D getOrCreatePositionData(Type type)
      {
         PositionData3D positionData = positionDataMap.get(type);

         if (positionData == null)
         {
            positionData = new PositionData3D(namePrefix, type, registry);
            positionDataMap.put(type, positionData);
            clearableData.add(positionData);
         }

         return positionData;
      }

      public QuaternionData3D getOrCreateOrientationData(Type type)
      {
         QuaternionData3D orientationData = orientationDataMap.get(type);

         if (orientationData == null)
         {
            orientationData = new QuaternionData3D(namePrefix, type, registry);
            orientationDataMap.put(type, orientationData);
            clearableData.add(orientationData);
         }

         return orientationData;
      }

      public VectorData3D getOrCreateVectorData3D(Type type, SpaceData3D space)
      {
         EnumMap<SpaceData3D, VectorData3D> vectorDataSubMap = getSubEnumMap(vectorDataMap, type, SpaceData3D.class);
         VectorData3D vectorData = vectorDataSubMap.get(space);

         if (vectorData == null)
         {
            vectorData = new VectorData3D(namePrefix, type, space, registry);
            vectorDataSubMap.put(space, vectorData);
            clearableData.add(vectorData);
         }

         return vectorData;
      }

      public AlphaFilteredVectorData3D getOrCreateAlphaFilteredVectorData(Type type, SpaceData3D space, DoubleProvider breakFrequency, double dt)
      {
         EnumMap<SpaceData3D, AlphaFilteredVectorData3D> filteredVectorDataSubMap = getSubEnumMap(filteredVectorDataMap, type, SpaceData3D.class);
         AlphaFilteredVectorData3D filteredVectorData = filteredVectorDataSubMap.get(space);

         if (filteredVectorData == null)
         {
            VectorData3D rawVectorData = getOrCreateVectorData3D(type, space);
            filteredVectorData = new AlphaFilteredVectorData3D(namePrefix, type, space, breakFrequency, dt, rawVectorData, registry);
            filteredVectorDataSubMap.put(space, filteredVectorData);
            clearableData.add(filteredVectorData);
         }

         return filteredVectorData;
      }

      public RateLimitedVectorData3D getOrCreateRateLimitedVectorData(Type type, SpaceData3D space, DoubleProvider maximumRate, double dt)
      {
         EnumMap<SpaceData3D, RateLimitedVectorData3D> rateLimitedVectorDataSubMap = getSubEnumMap(rateLimitedVectorDataMap, type, SpaceData3D.class);
         RateLimitedVectorData3D rateLimitedVectorData = rateLimitedVectorDataSubMap.get(space);

         if (rateLimitedVectorData == null)
         {
            VectorData3D rawVectorData = getOrCreateVectorData3D(type, space);
            rateLimitedVectorData = new RateLimitedVectorData3D(namePrefix, type, space, maximumRate, dt, rawVectorData, registry);
            rateLimitedVectorDataSubMap.put(space, rateLimitedVectorData);
            clearableData.add(rateLimitedVectorData);
         }

         return rateLimitedVectorData;
      }

      public YoPID3DGains getOrCreateOrientationGains(boolean useIntegrator)
      {
         if (orientationGains == null)
            orientationGains = new DefaultYoPID3DGains(namePrefix + "Orientation", GainCoupling.NONE, useIntegrator, registry);
         return orientationGains;
      }

      public YoPID3DGains getOrCreatePositionGains(boolean useIntegrator)
      {
         if (positionGains == null)
            positionGains = new DefaultYoPID3DGains(namePrefix + "Position", GainCoupling.NONE, useIntegrator, registry);
         return positionGains;
      }

      public YoSE3OffsetFrame getOrCreateControlFrame(ReferenceFrame parentFrame)
      {
         if (controlFrame == null)
            controlFrame = new YoSE3OffsetFrame(namePrefix + "BodyFixedControlFrame", parentFrame, registry);
         return controlFrame;
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
