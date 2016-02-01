package us.ihmc.darpaRoboticsChallenge.wholeBodyInverseKinematicsSimulationController;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ComputeOption;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ComputeResult;

public abstract class WholeBodyIKSimController implements RobotController
{
   private final SDFRobot scsRobot;
   private final SDFFullHumanoidRobotModel actualRobotModel;
   private final SDFFullRobotModel desiredRobotModel;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   //   private OneDoFJoint[] joints;

   private final JointAnglesWriter jointAnglesWriter;

   private final Vector3d positionInWorld = new Vector3d();
   private final Quat4d rotation = new Quat4d();

   //   private final WholeBodyControlParameters drcRobotModel;

   private final Vector3d offset = new Vector3d();
   private double initialYaw;
   private final RigidBodyTransform rootToWorld = new RigidBodyTransform();
   private final DRCRobotModel drcRobotModel;

   protected final ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>> revoluteJoints = new ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>>();
   private final DenseMatrix64F a =  new DenseMatrix64F(4,4);
   private final DenseMatrix64F b =  new DenseMatrix64F(4,4);
   private final DenseMatrix64F c =  new DenseMatrix64F(4,4);
   private final DenseMatrix64F d =  new DenseMatrix64F(4,4);
   private final DoubleYoVariable x_offset_for_IK = new DoubleYoVariable("x_offset_for_IK", registry);
   private final DoubleYoVariable y_offset_for_IK = new DoubleYoVariable("y_offset_for_IK", registry);
   private final DoubleYoVariable z_offset_for_IK = new DoubleYoVariable("z_offset_for_IK", registry);
   private final DoubleYoVariable x_offset_from_pelvis = new DoubleYoVariable("x_offset_from_pelvis", registry);
   private final DoubleYoVariable y_offset_from_pelvis = new DoubleYoVariable("y_offset_from_pelvis", registry);
   private final DoubleYoVariable z_offset_from_pelvis = new DoubleYoVariable("z_offset_from_pelvis", registry);
   private final boolean DEBUG = true;
   private SDFFullRobotModel initialFullRobotModel;
   private RigidBodyTransform footToWorld;
   private WholeBodyIkSolver wholeBodyIkSolver;
   
   public WholeBodyIKSimController(SDFRobot robot, SDFFullHumanoidRobotModel actualRobotModel, DRCRobotModel drcRobotModel) throws Exception
   {
      this.scsRobot = robot;
      this.actualRobotModel = actualRobotModel;
      this.drcRobotModel = drcRobotModel;
      this.desiredRobotModel = drcRobotModel.createFullRobotModel();

      jointAnglesWriter = new JointAnglesWriter(scsRobot, actualRobotModel);
      try
      {
         wholeBodyIkSolver = drcRobotModel.createWholeBodyIkSolver();
         wholeBodyIkSolver.getConfiguration().setNumberOfControlledDoF(RobotSide.RIGHT, WholeBodyIkSolver.ControlledDoF.DOF_3P);
         wholeBodyIkSolver.getConfiguration().setNumberOfControlledDoF(RobotSide.LEFT, WholeBodyIkSolver.ControlledDoF.DOF_NONE);
         wholeBodyIkSolver.getHierarchicalSolver().setVerbosityLevel(0);    
      }
      catch (Exception e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      initialize();
   }
   
   public abstract void setInitialJointAngles(SDFRobot scsRobot, SDFFullRobotModel fullRobotModel, DRCRobotModel drcRobotModel);
   
   public abstract ArrayList<ImmutablePair<RobotSide, RigidBodyTransform>> getTargetsQueue();

   
   @Override
   public void initialize()
   {
      setInitialJointAngles(scsRobot, actualRobotModel, drcRobotModel);
      scsRobot.update();
      
      ReferenceFrame footFrame = actualRobotModel.getFoot(RobotSide.LEFT).getBodyFixedFrame();
      footToWorld = new RigidBodyTransform();
      footFrame.getTransformToDesiredFrame(footToWorld, actualRobotModel.getWorldFrame());
      
      scsRobot.getRootJointToWorldTransform(rootToWorld);
      rootToWorld.get(rotation, positionInWorld);
      
      double pelvisToFoot = 0.887;
      
      positionInWorld.setZ(pelvisToFoot);
      offset.setZ(0.06);
      positionInWorld.add(offset);
      
      scsRobot.setPositionInWorld(positionInWorld);
      actualRobotModel.getRootJoint().setPosition(positionInWorld);
      
      FrameOrientation frameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), rotation);
      double[] yawPitchRoll = frameOrientation.getYawPitchRoll();
      yawPitchRoll[0] = initialYaw;
      frameOrientation.setYawPitchRoll(yawPitchRoll);
      
      scsRobot.setOrientation(frameOrientation.getQuaternionCopy());
      scsRobot.update();
      
      actualRobotModel.getRootJoint().setRotation(frameOrientation.getQuaternionCopy());
      actualRobotModel.updateFrames();


      jointAnglesWriter.write();
     
      initialFullRobotModel = actualRobotModel;
      
      //Good initial positions for Atlas
      x_offset_for_IK.set(0.32364);
      y_offset_for_IK.set(-0.35768);
      z_offset_for_IK.set(0.75860);

   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
//      RigidBodyTransform desiredToWorldTransform = new RigidBodyTransform();
//      desiredToWorldTransform.setTranslation(x_offset_for_IK.getDoubleValue(), y_offset_for_IK.getDoubleValue(), z_offset_for_IK.getDoubleValue());
      
      Point3d point = new Point3d( x_offset_for_IK.getDoubleValue(), y_offset_for_IK.getDoubleValue(), z_offset_for_IK.getDoubleValue());
      FramePose desiredPose = new FramePose(ReferenceFrame.getWorldFrame(),point, new Quat4d() ); 
      
      wholeBodyIkSolver.setGripperPalmTarget( RobotSide.RIGHT, desiredPose);

      ComputeResult ret = ComputeResult.FAILED_INVALID;
      try
      {
         ret = wholeBodyIkSolver.compute(actualRobotModel,  desiredRobotModel, ComputeOption.USE_ACTUAL_MODEL_JOINTS);
      }
      catch (Exception e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      
      if(DEBUG){
         System.out.println("ret: " + ret);
      }
      
//      fullRobotModel.getOneDoFJointByName(drcRobotModel.getJointMap().getArmJointName(RobotSide.RIGHT, ArmJointName.SHOULDER_YAW)).setQ(x_offset_for_IK.getDoubleValue()); //arm_shy
//      fullRobotModel.getOneDoFJointByName(drcRobotModel.getJointMap().getArmJointName(RobotSide.RIGHT, ArmJointName.SHOULDER_ROLL)).setQ(y_offset_for_IK.getDoubleValue()); //arm_shx
//      fullRobotModel.getOneDoFJointByName(drcRobotModel.getJointMap().getArmJointName(RobotSide.RIGHT, ArmJointName.ELBOW_PITCH)).setQ(z_offset_for_IK.getDoubleValue()); //arm_ely
//      fullRobotModel.getOneDoFJointByName(drcRobotModel.getJointMap().getArmJointName(RobotSide.RIGHT, ArmJointName.ELBOW_ROLL)).setQ(-0.498); //arm_elx
//      fullRobotModel.getOneDoFJointByName(drcRobotModel.getJointMap().getArmJointName(RobotSide.RIGHT, ArmJointName.FIRST_WRIST_PITCH)).setQ(0.000); //arm_wry
//      fullRobotModel.getOneDoFJointByName(drcRobotModel.getJointMap().getArmJointName(RobotSide.RIGHT, ArmJointName.WRIST_ROLL)).setQ(0.004); //arm_wrx
//
//      fullRobotModel.getEndEffectorFrame(RobotSide.RIGHT, LimbName.ARM).getTransformToDesiredFrame(fullRobotModel.getWorldFrame()).get(d); //getPelvis().getBodyFixedFrame()).get(d);
//      x_offset_from_pelvis.set(d.get(0,3));
//      y_offset_from_pelvis.set(d.get(1,3));
//      z_offset_from_pelvis.set(d.get(2,3));
      
//      jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
      jointAnglesWriter.write();
      
   }

   

}