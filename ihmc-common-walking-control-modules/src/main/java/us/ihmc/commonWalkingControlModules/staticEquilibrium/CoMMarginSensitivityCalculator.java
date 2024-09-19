package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.inverseKinematics.RobotJointVelocityAccelerationIntegrator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLineSegment2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import static us.ihmc.commonWalkingControlModules.staticEquilibrium.CenterOfMassStabilityMarginRegionCalculator.*;

// TODO should be removed before finalizing
public class CoMMarginSensitivityCalculator
{
   public static final double DT = 1.0e-2;

   private final CenterOfMassStabilityMarginOptimizationModule optimizationModule;
   private final RobotJointVelocityAccelerationIntegrator integrator = new RobotJointVelocityAccelerationIntegrator(DT);
   private final FullHumanoidRobotModel fullRobotModel;
   private final JointBasics[] controlledJoints;
   private final WholeBodyContactState contactState;

   // TODO change to give option to operate on the same robot model?
   public CoMMarginSensitivityCalculator(FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory)
   {
      fullRobotModel = fullHumanoidRobotModelFactory.createFullRobotModel();
      this.optimizationModule = new CenterOfMassStabilityMarginOptimizationModule(fullRobotModel.getTotalMass());
      controlledJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel);
      OneDoFJointBasics[] controlledOneDoFJoints = MultiBodySystemTools.filterJoints(controlledJoints, OneDoFJointBasics.class);
      contactState = new WholeBodyContactState(controlledOneDoFJoints, fullRobotModel.getRootJoint());
   }

   public void setRobotState(JointBasics[] source)
   {
      for (int jointIndex = 0; jointIndex < source.length; jointIndex++)
      {
         JointReadOnly sourceJoint = source[jointIndex];
         JointBasics destinationJoint = controlledJoints[jointIndex];
         destinationJoint.setJointConfiguration(sourceJoint);
      }

      fullRobotModel.updateFrames();
   }

   public void integrateRobotState(DMatrixRMaj velocity)
   {
      integrator.integrateJointVelocities(controlledJoints, velocity);
      MultiBodySystemTools.insertJointsState(controlledJoints, JointStateType.CONFIGURATION, integrator.getJointConfigurations());
      fullRobotModel.updateFrames();
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public WholeBodyContactState getContactState()
   {
      return contactState;
   }

   public Point2D computeVertex(TIntArrayList basisIndices)
   {
      optimizationModule.updateContactState(contactState);
      return optimizationModule.solveForFixedBasis(basisIndices);
   }
}