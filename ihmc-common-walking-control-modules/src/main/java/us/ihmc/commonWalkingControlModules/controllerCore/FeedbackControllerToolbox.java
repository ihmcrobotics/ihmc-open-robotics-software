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
import us.ihmc.commonWalkingControlModules.controllerCore.data.FeedbackControllerData;
import us.ihmc.commonWalkingControlModules.controllerCore.data.PositionData3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.QuaternionData3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.RateLimitedVectorData3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.Space;
import us.ihmc.commonWalkingControlModules.controllerCore.data.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.data.VectorData3D;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
import us.ihmc.robotics.dataStructures.YoMutableFrameSpatialVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoMutableFrameSpatialVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoMutableFrameVector3D;
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
 */
public class FeedbackControllerToolbox implements FeedbackControllerDataHolderReadOnly
{
   public static final String centerOfMassName = "centerOfMass";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private SingleFeedbackControllerDataPool centerOfMassDataPool;
   private final Map<RigidBodyBasics, SingleFeedbackControllerDataPool> endEffectorDataPoolMap = new HashMap<>();
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
         centerOfMassDataPool = new SingleFeedbackControllerDataPool(centerOfMassName, registry);
         singleFeedbackControllerDataPoolList.add(centerOfMassDataPool);
      }
      return centerOfMassDataPool;
   }

   /**
    * Retrieves and returns the {@code YoMutableFramePoint3D} for the center of mass associated with
    * the given {@code type}, if it does not exist it is created.
    *
    * @param type the type of the data to retrieve.
    * @return the unique {@code YoMutableFramePoint3D} matching the search criterion.
    */
   public YoMutableFramePoint3D getOrCreateCenterOfMassPositionData(Type type, YoBoolean enabled)
   {
      PositionData3D positionData = getOrCreateCenterOfMassDataPool().getOrCreatePositionData(type);
      positionData.addActiveFlag(enabled);
      return positionData;
   }

   /**
    * Retrieves and returns the {@code YoMutableFrameVector3D} for the center of mass associated with
    * the given {@code type}, and {@code space}, if it does not exist it is created.
    *
    * @param type  the type of the data to retrieve.
    * @param space the space of the data to retrieve.
    * @return the unique {@code YoMutableFrameVector3D} matching the search criteria.
    */
   public YoMutableFrameVector3D getOrCreateCenterOfMassVectorData(Type type, Space space, YoBoolean enabled)
   {
      VectorData3D vectorData = getOrCreateCenterOfMassDataPool().getOrCreateVectorData(type, space);
      vectorData.addActiveFlag(enabled);
      return vectorData;
   }

   /**
    * Retrieves and returns the {@code AlphaFilteredYoMutableFrameVector3D} for the center of mass
    * associated with the given {@code type} and {@code space}, if it does not exist it is created.
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
    * @return the unique {@code AlphaFilteredYoMutableFrameVector3D} matching the search criteria.
    */
   public AlphaFilteredYoMutableFrameVector3D getOrCreateCenterOfMassAlphaFilteredVectorData(Type rawDataType, Space space, double dt,
                                                                                             DoubleProvider breakFrequencyProvider, YoBoolean enabled)
   {
      AlphaFilteredVectorData3D filteredVectorData = getOrCreateCenterOfMassDataPool().getOrCreateAlphaFilteredVectorData(rawDataType,
                                                                                                                          space,
                                                                                                                          breakFrequencyProvider,
                                                                                                                          dt);
      filteredVectorData.addActiveFlag(enabled);
      return filteredVectorData;
   }

   /**
    * Retrieves and returns the {@code RateLimitedYoMutableFrameVector3D} for the center of mass
    * associated with the given {@code type}, and {@code space}, if it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt} and {@code maximumRate} are only used if the data does not exist
    * yet.
    * </p>
    *
    * @param space       the space of the data to retrieve.
    * @param rawDataType the type of the raw vector onto which the rate limit is to be applied.
    * @param dt          the duration of a control tick.
    * @param maximumRate the maximum rate allowed rate. Not modified.
    * @return the unique {@code RateLimitedYoMutableFrameVector3D} matching the search criteria.
    */
   public RateLimitedYoMutableFrameVector3D getOrCreateCenterOfMassRateLimitedVectorData(Type rawDataType, Space space, double dt, YoDouble maximumRate,
                                                                                         YoBoolean enabled)
   {
      RateLimitedVectorData3D rateLimitedVectorData = getOrCreateCenterOfMassDataPool().getOrCreateRateLimitedVectorData(rawDataType, space, maximumRate, dt);
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
      return getOrCreateCenterOfMassDataPool().getOrCreatePositionGains(useIntegrator);
   }

   private SingleFeedbackControllerDataPool getOrCreateEndEffectorDataPool(RigidBodyBasics endEffector)
   {
      SingleFeedbackControllerDataPool endEffectorDataPool = endEffectorDataPoolMap.get(endEffector);
      if (endEffectorDataPool == null)
      {
         endEffectorDataPool = new SingleFeedbackControllerDataPool(endEffector.getName(), registry);
         endEffectorDataPoolMap.put(endEffector, endEffectorDataPool);
         singleFeedbackControllerDataPoolList.add(endEffectorDataPool);
      }
      return endEffectorDataPool;
   }

   /**
    * Retrieves and returns the {@code YoMutableFramePoint3D} associated with the given end-effector
    * and {@code type}, if it does not exist it is created.
    * <p>
    * The name prefix of the created variable is created as follows:<br>
    * {@code namePrefix = endEffector.getName() + type.getName() +}
    * {@link Space#POSITION}{@code .getName()}<br>
    * Such that the desired position for the rigid-body 'rightHand' will have the prefix:
    * "rightHandDesiredPosition".
    * </p>
    *
    * @param endEffector the end-effector to which the returned data is associated.
    * @param type        the type of the data to retrieve.
    * @return the unique {@code YoMutableFramePoint3D} matching the search criteria.
    */
   public YoMutableFramePoint3D getOrCreatePositionData(RigidBodyBasics endEffector, Type type, YoBoolean enabled)
   {
      PositionData3D positionData = getOrCreateEndEffectorDataPool(endEffector).getOrCreatePositionData(type);
      positionData.addActiveFlag(enabled);
      return positionData;
   }

   /**
    * Retrieves and returns the {@code YoMutableFrameQuaternion} associated with the given end-effector
    * and {@code type}, if it does not exist it is created.
    * <p>
    * The name prefix of the created variable is created as follows:<br>
    * {@code namePrefix = endEffector.getName() + type.getName() +}
    * {@link Space#ORIENTATION}{@code .getName()}<br>
    * Such that the current orientation for the rigid-body 'rightHand' will have the prefix:
    * "rightHandCurrentOrientation".
    * </p>
    *
    * @param endEffector the end-effector to which the returned data is associated.
    * @param type        the type of the data to retrieve.
    * @return the unique {@code YoMutableFrameQuaternion} matching the search criteria.
    */
   public YoMutableFrameQuaternion getOrCreateOrientationData(RigidBodyBasics endEffector, Type type, YoBoolean enabled)
   {
      QuaternionData3D orientationData = getOrCreateEndEffectorDataPool(endEffector).getOrCreateOrientationData(type);
      orientationData.addActiveFlag(enabled);
      return orientationData;
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
    * @param type        the type of the data to retrieve.
    * @param space       the space of the data to retrieve.
    * @return the unique {@code YoMutableFrameVector3D} matching the search criteria.
    */
   public YoMutableFrameVector3D getOrCreateVectorData(RigidBodyBasics endEffector, Type type, Space space, YoBoolean enabled)
   {
      VectorData3D vectorData = getOrCreateEndEffectorDataPool(endEffector).getOrCreateVectorData(type, space);
      vectorData.addActiveFlag(enabled);
      return vectorData;
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
    * @param space       the space of the data to retrieve.
    * @param rawDataType the type of the raw vector onto which the rate limit is to be applied.
    * @param dt          the duration of a control tick.
    * @param maximumRate the maximum rate allowed rate. Not modified.
    * @return the unique {@code RateLimitedYoMutableFrameVector3D} matching the search criteria.
    */
   public RateLimitedYoMutableFrameVector3D getOrCreateRateLimitedVectorData(RigidBodyBasics endEffector, Type rawDataType, Space space, double dt,
                                                                             YoDouble maximumRate, YoBoolean enabled)
   {
      RateLimitedVectorData3D rateLimitedVectorData = getOrCreateEndEffectorDataPool(endEffector).getOrCreateRateLimitedVectorData(rawDataType,
                                                                                                                                   space,
                                                                                                                                   maximumRate,
                                                                                                                                   dt);
      rateLimitedVectorData.addActiveFlag(enabled);
      return rateLimitedVectorData;
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
    * @param endEffector            the end-effector to which the returned data is associated.
    * @param space                  the space of the data to retrieve.
    * @param rawDataType            the type of the raw vector onto which the rate limit is to be
    *                               applied.
    * @param dt                     the duration of a control tick.
    * @param breakFrequencyProvider the break frequency to use for the low-pass filter. Not modified.
    * @return the unique {@code AlphaFilteredYoMutableFrameVector3D} matching the search criteria.
    */
   public AlphaFilteredYoMutableFrameVector3D getOrCreateAlphaFilteredVectorData(RigidBodyBasics endEffector, Type rawDataType, Space space, double dt,
                                                                                 DoubleProvider breakFrequencyProvider, YoBoolean enabled)
   {
      AlphaFilteredVectorData3D alphaFilteredVectorData = getOrCreateEndEffectorDataPool(endEffector).getOrCreateAlphaFilteredVectorData(rawDataType,
                                                                                                                                         space,
                                                                                                                                         breakFrequencyProvider,
                                                                                                                                         dt);
      alphaFilteredVectorData.addActiveFlag(enabled);
      return alphaFilteredVectorData;
   }

   /**
    * Retrieves and returns the {@code YoMutableFramePose3D} associated with the given end-effector and
    * {@code type}, if it does not exist it is created.
    *
    * @param endEffector the end-effector to which the returned data is associated.
    * @param type        the type of the data to retrieve.
    * @return the unique {@code YoMutableFramePose3D} matching the search criteria.
    */
   public YoMutableFramePose3D getOrCreatePoseData(RigidBodyBasics endEffector, Type type, YoBoolean enabled)
   {
      return new YoMutableFramePose3D(getOrCreatePositionData(endEffector, type, enabled), getOrCreateOrientationData(endEffector, type, enabled));
   }

   /**
    * Retrieves and returns the {@code YoMutableFrameSpatialVector} for holding the angular and linear
    * velocities of the given end-effector for representing a given data {@code type}. If it does not
    * exist it is created.
    *
    * @param endEffector the end-effector to which the returned data is associated.
    * @param type        the type of the data to retrieve.
    * @return the unique {@code YoMutableFrameSpatialVector} matching the search criteria.
    */
   public YoMutableFrameSpatialVector getOrCreateSpatialVelocityData(RigidBodyBasics endEffector, Type type, YoBoolean enabled)
   {
      return new YoMutableFrameSpatialVector(getOrCreateVectorData(endEffector, type, Space.ANGULAR_VELOCITY, enabled),
                                             getOrCreateVectorData(endEffector, type, Space.LINEAR_VELOCITY, enabled));
   }

   /**
    * Retrieves and returns the {@code YoMutableFrameSpatialVector} for holding the angular and linear
    * accelerations of the given end-effector for representing a given data {@code type}. If it does
    * not exist it is created.
    *
    * @param endEffector the end-effector to which the returned data is associated.
    * @param type        the type of the data to retrieve.
    * @return the unique {@code YoMutableFrameSpatialVector} matching the search criteria.
    */
   public YoMutableFrameSpatialVector getOrCreateSpatialAccelerationData(RigidBodyBasics endEffector, Type type, YoBoolean enabled)
   {
      return new YoMutableFrameSpatialVector(getOrCreateVectorData(endEffector, type, Space.ANGULAR_ACCELERATION, enabled),
                                             getOrCreateVectorData(endEffector, type, Space.LINEAR_ACCELERATION, enabled));
   }

   /**
    * Retrieves and returns the {@code YoMutableFrameSpatialVector} for holding the angular and linear
    * forces of the given end-effector for representing a given data {@code type}. If it does not exist
    * it is created.
    *
    * @param endEffector the end-effector to which the returned data is associated.
    * @param type        the type of the data to retrieve.
    * @return the unique {@code YoMutableFrameSpatialVector} matching the search criteria.
    */
   public YoMutableFrameSpatialVector getOrCreateSpatialForceData(RigidBodyBasics endEffector, Type type, YoBoolean enabled)
   {
      return new YoMutableFrameSpatialVector(getOrCreateVectorData(endEffector, type, Space.ANGULAR_TORQUE, enabled),
                                             getOrCreateVectorData(endEffector, type, Space.LINEAR_FORCE, enabled));
   }

   /**
    * Retrieves and returns the {@code RateLimitedYoMutableSpatialVector} for the rate-limited angular
    * and linear velocities of the given end-effector. The data type of the vector is defined by
    * {@code type}. If it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt}, {@code maximumLinearRate}, and {@code maximumAngularRate} are
    * only used if the data does not exist yet.
    * </p>
    *
    * @param endEffector        the end-effector to which the returned data is associated.
    * @param rawDataType        the type of the raw vector onto which the rate limit is to be applied.
    * @param dt                 the duration of a control tick.
    * @param maximumAngularRate the maximum angular rate allowed rate. Not modified.
    * @param maximumLinearRate  the maximum linear rate allowed rate. Not modified.
    * @return the unique {@code RateLimitedYoMutableSpatialVector} matching the search criteria.
    */
   public RateLimitedYoMutableSpatialVector getOrCreateRateLimitedSpatialVelocityData(RigidBodyBasics endEffector, Type rawDataType, double dt,
                                                                                      YoDouble maximumAngularRate, YoDouble maximumLinearRate,
                                                                                      YoBoolean enabled)
   {
      return new RateLimitedYoMutableSpatialVector(getOrCreateRateLimitedVectorData(endEffector,
                                                                                    rawDataType,
                                                                                    Space.ANGULAR_VELOCITY,
                                                                                    dt,
                                                                                    maximumAngularRate,
                                                                                    enabled),
                                                   getOrCreateRateLimitedVectorData(endEffector,
                                                                                    rawDataType,
                                                                                    Space.LINEAR_VELOCITY,
                                                                                    dt,
                                                                                    maximumLinearRate,
                                                                                    enabled));
   }

   /**
    * Retrieves and returns the {@code AlphaFilteredYoMutableFrameSpatialVector} for the filtered
    * angular and linear velocity errors of the given end-effector. If it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt}, {@code breakFrequencyLinearPart}, and
    * {@code breakFrequencyAngularPart} are only used if the data does not exist yet.
    * </p>
    *
    * @param endEffector               the end-effector to which the returned data is associated.
    * @param rawDataType               the type of the raw vector onto which the filter is to be
    *                                  applied.
    * @param dt                        the duration of a control tick.
    * @param breakFrequencyAngularPart the break frequency to use for the angular part of the velocity
    *                                  error. Not modified.
    * @param breakFrequencyLinearPart  the break frequency to use for the linear part of the velocity
    *                                  error. Not modified.
    * @return the unique {@code AlphaFilteredYoMutableFrameSpatialVector} matching the search criteria.
    */
   public AlphaFilteredYoMutableFrameSpatialVector getOrCreateAlphaFilteredSpatialVelocityData(RigidBodyBasics endEffector, Type rawDataType, double dt,
                                                                                               DoubleProvider breakFrequencyAngularPart,
                                                                                               DoubleProvider breakFrequencyLinearPart, YoBoolean enabled)
   {
      return new AlphaFilteredYoMutableFrameSpatialVector(getOrCreateAlphaFilteredVectorData(endEffector,
                                                                                             rawDataType,
                                                                                             Space.ANGULAR_VELOCITY,
                                                                                             dt,
                                                                                             breakFrequencyAngularPart,
                                                                                             enabled),
                                                          getOrCreateAlphaFilteredVectorData(endEffector,
                                                                                             rawDataType,
                                                                                             Space.LINEAR_VELOCITY,
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
    * @param rawDataType        the type of the raw vector onto which the rate limit is to be applied.
    * @param dt                 the duration of a control tick.
    * @param maximumAngularRate the maximum angular rate allowed rate. Not modified.
    * @param maximumLinearRate  the maximum linear rate allowed rate. Not modified.
    * @return the unique {@code RateLimitedYoMutableSpatialVector} matching the search criteria.
    */
   public RateLimitedYoMutableSpatialVector getOrCreateRateLimitedSpatialAccelerationData(RigidBodyBasics endEffector, Type rawDataType, double dt,
                                                                                          YoDouble maximumAngularRate, YoDouble maximumLinearRate,
                                                                                          YoBoolean enabled)
   {
      return new RateLimitedYoMutableSpatialVector(getOrCreateRateLimitedVectorData(endEffector,
                                                                                    rawDataType,
                                                                                    Space.ANGULAR_ACCELERATION,
                                                                                    dt,
                                                                                    maximumAngularRate,
                                                                                    enabled),
                                                   getOrCreateRateLimitedVectorData(endEffector,
                                                                                    rawDataType,
                                                                                    Space.LINEAR_ACCELERATION,
                                                                                    dt,
                                                                                    maximumLinearRate,
                                                                                    enabled));
   }

   /**
    * Retrieves and returns the {@code RateLimitedYoMutableSpatialVector} for the rate-limited angular
    * and linear accelerations of the given end-effector. The date type of the vector is defined by
    * {@code type}. If it does not exist it is created.
    * <p>
    * Note: the arguments {@code dt}, {@code maximumLinearRate}, and {@code maximumAngularRate} are
    * only used if the data does not exist yet.
    * </p>
    *
    * @param endEffector        the end-effector to which the returned data is associated.
    * @param rawDataType        the type of the raw vector onto which the rate limit is to be applied.
    * @param dt                 the duration of a control tick.
    * @param maximumAngularRate the maximum angular rate allowed rate. Not modified.
    * @param maximumLinearRate  the maximum linear rate allowed rate. Not modified.
    * @return the unique {@code RateLimitedYoMutableSpatialVector} matching the search criteria.
    */
   public RateLimitedYoMutableSpatialVector getOrCreateRateLimitedSpatialForceData(RigidBodyBasics endEffector, Type rawDataType, double dt,
                                                                                   YoDouble maximumAngularRate, YoDouble maximumLinearRate, YoBoolean enabled)
   {
      return new RateLimitedYoMutableSpatialVector(getOrCreateRateLimitedVectorData(endEffector,
                                                                                    rawDataType,
                                                                                    Space.ANGULAR_TORQUE,
                                                                                    dt,
                                                                                    maximumAngularRate,
                                                                                    enabled),
                                                   getOrCreateRateLimitedVectorData(endEffector,
                                                                                    rawDataType,
                                                                                    Space.LINEAR_FORCE,
                                                                                    dt,
                                                                                    maximumLinearRate,
                                                                                    enabled));
   }

   /**
    * Retrieves and returns the set of orientation gains {@code YoPID3DGains} associated to the given
    * end-effector, if it does not exist it is created.
    *
    * @param endEffector   the end-effector to which the gains are associated.
    * @param useIntegrator whether to create the gains necessary to compute the integral term.
    * @return the unique {@code YoPID3DGains} associated with the given end-effector.
    */
   public YoPID3DGains getOrCreateOrientationGains(RigidBodyBasics endEffector, boolean useIntegrator)
   {
      return getOrCreateEndEffectorDataPool(endEffector).getOrCreateOrientationGains(useIntegrator);
   }

   /**
    * Retrieves and returns the set of position gains {@code YoPID3DGains} associated to the given
    * end-effector, if it does not exist it is created.
    *
    * @param endEffector   the end-effector to which the gains are associated.
    * @param useIntegrator whether to create the gains necessary to compute the integral term.
    * @return the unique {@code YoPID3DGains} associated with the given end-effector.
    */
   public YoPID3DGains getOrCreatePositionGains(RigidBodyBasics endEffector, boolean useIntegrator)
   {
      return getOrCreateEndEffectorDataPool(endEffector).getOrCreatePositionGains(useIntegrator);
   }

   /**
    * Retrieves and returns the set of gains {@code YoPIDSE3Gains} associated to the given
    * end-effector, if it does not exist it is created.
    *
    * @param endEffector   the end-effector to which the gains are associated.
    * @param useIntegrator whether to create the gains necessary to compute the integral term.
    * @return the unique {@code YoPIDSE3Gains} associated with the given end-effector.
    */
   public YoPIDSE3Gains getOrCreateSE3PIDGains(RigidBodyBasics endEffector, boolean useIntegrator)
   {
      YoPID3DGains positionGains = getOrCreatePositionGains(endEffector, useIntegrator);
      YoPID3DGains orientationGains = getOrCreateOrientationGains(endEffector, useIntegrator);
      return new DefaultYoPIDSE3Gains(positionGains, orientationGains);
   }

   /**
    * Retrieves and returns the control frame {@code YoSE3OffsetFrame} associated to the given
    * end-effector, if it does not exist it is created.
    *
    * @param endEffector the end-effector to which the control frame is associated.
    * @return the unique {@code YoSE3OffsetFrame} control frame associated with the given end-effector.
    */
   public YoSE3OffsetFrame getOrCreateControlFrame(RigidBodyBasics endEffector)
   {
      return getOrCreateEndEffectorDataPool(endEffector).getOrCreateControlFrame(endEffector.getBodyFixedFrame());
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
   public void getCenterOfMassVectorData(List<VectorData3D> vectorDataListToPack, Type type, Space space)
   {
      vectorDataListToPack.clear();
      EnumMap<Space, VectorData3D> endEffectorDataTyped = getOrCreateCenterOfMassDataPool().vectorDataMap.get(type);

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
      EnumMap<Type, PositionData3D> endEffectorData = getOrCreateEndEffectorDataPool(endEffector).positionDataMap;

      if (endEffectorData == null)
         return;

      PositionData3D positionData = endEffectorData.get(type);

      if (positionData == null || !positionData.isActive())
         return;

      positionDataListToPack.add(positionData);
   }

   @Override
   public void getOrientationData(RigidBodyBasics endEffector, List<QuaternionData3D> orientationDataListToPack, Type type)
   {
      orientationDataListToPack.clear();
      EnumMap<Type, QuaternionData3D> endEffectorData = getOrCreateEndEffectorDataPool(endEffector).orientationDataMap;

      if (endEffectorData == null)
         return;

      QuaternionData3D orientationData = endEffectorData.get(type);

      if (orientationData == null || !orientationData.isActive())
         return;

      orientationDataListToPack.add(orientationData);
   }

   @Override
   public void getVectorData(RigidBodyBasics endEffector, List<VectorData3D> vectorDataListToPack, Type type, Space space)
   {
      vectorDataListToPack.clear();
      EnumMap<Type, EnumMap<Space, VectorData3D>> endEffectorData = getOrCreateEndEffectorDataPool(endEffector).vectorDataMap;

      if (endEffectorData == null)
         return;

      EnumMap<Space, VectorData3D> endEffectorDataTyped = endEffectorData.get(type);

      if (endEffectorDataTyped == null)
         return;

      VectorData3D vectorData = endEffectorDataTyped.get(space);

      if (vectorData == null || !vectorData.isActive())
         return;

      vectorDataListToPack.add(vectorData);
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

   private static class SingleFeedbackControllerDataPool
   {
      private final YoVariableRegistry registry;
      private final String namePrefix;
      private final EnumMap<Type, PositionData3D> positionDataMap = new EnumMap<>(Type.class);
      private final EnumMap<Type, QuaternionData3D> orientationDataMap = new EnumMap<>(Type.class);
      private final EnumMap<Type, EnumMap<Space, VectorData3D>> vectorDataMap = new EnumMap<>(Type.class);
      private final EnumMap<Type, EnumMap<Space, RateLimitedVectorData3D>> rateLimitedVectorDataMap = new EnumMap<>(Type.class);
      private final EnumMap<Type, EnumMap<Space, AlphaFilteredVectorData3D>> filteredVectorDataMap = new EnumMap<>(Type.class);

      private YoPID3DGains orientationGains;
      private YoPID3DGains positionGains;
      private YoSE3OffsetFrame controlFrame;

      private final List<FeedbackControllerData> clearableData = new ArrayList<>();

      public SingleFeedbackControllerDataPool(String namePrefix, YoVariableRegistry registry)
      {
         this.namePrefix = namePrefix;
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

      public VectorData3D getOrCreateVectorData(Type type, Space space)
      {
         EnumMap<Space, VectorData3D> vectorDataSubMap = getSubEnumMap(vectorDataMap, type, Space.class);
         VectorData3D vectorData = vectorDataSubMap.get(space);

         if (vectorData == null)
         {
            vectorData = new VectorData3D(namePrefix, type, space, registry);
            vectorDataSubMap.put(space, vectorData);
            clearableData.add(vectorData);
         }

         return vectorData;
      }

      public AlphaFilteredVectorData3D getOrCreateAlphaFilteredVectorData(Type type, Space space, DoubleProvider breakFrequency, double dt)
      {
         EnumMap<Space, AlphaFilteredVectorData3D> filteredVectorDataSubMap = getSubEnumMap(filteredVectorDataMap, type, Space.class);
         AlphaFilteredVectorData3D filteredVectorData = filteredVectorDataSubMap.get(space);

         if (filteredVectorData == null)
         {
            VectorData3D rawVectorData = getOrCreateVectorData(type, space);
            filteredVectorData = new AlphaFilteredVectorData3D(namePrefix, type, space, breakFrequency, dt, rawVectorData, registry);
            filteredVectorDataSubMap.put(space, filteredVectorData);
            clearableData.add(filteredVectorData);
         }

         return filteredVectorData;
      }

      public RateLimitedVectorData3D getOrCreateRateLimitedVectorData(Type type, Space space, DoubleProvider maximumRate, double dt)
      {
         EnumMap<Space, RateLimitedVectorData3D> rateLimitedVectorDataSubMap = getSubEnumMap(rateLimitedVectorDataMap, type, Space.class);
         RateLimitedVectorData3D rateLimitedVectorData = rateLimitedVectorDataSubMap.get(space);

         if (rateLimitedVectorData == null)
         {
            VectorData3D rawVectorData = getOrCreateVectorData(type, space);
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
