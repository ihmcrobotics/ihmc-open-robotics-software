package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.HashMap;
import java.util.HashSet;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.Stoppable;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredJointsPositionProvider;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.communication.packets.wholebody.JointAnglesPacket;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.humanoidRobot.partNames.NeckJointName;
import us.ihmc.utilities.humanoidRobot.partNames.SpineJointName;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.controllers.PIDController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.trajectories.OneDoFJointQuinticTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;

public class JointPositionHighLevelController extends HighLevelBehavior implements Stoppable
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final HashSet<OneDoFJoint> jointsBeenControlled = new HashSet<OneDoFJoint>();
   private final FullRobotModel fullRobotModel;

   public final static HighLevelState controllerState = HighLevelState.JOINT_POSITION_CONTROL;

   private final HashMap<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator> trajectoryGenerator;
   private final HashMap<OneDoFJoint, PIDController >   alternativeController;
   private final HashMap<OneDoFJoint, BooleanYoVariable >   useAlternativeController;
   
   private final YoVariableDoubleProvider trajectoryTimeProvider;
   private final DoubleYoVariable timeProvider;

   private double initialTrajectoryTime;
   private final DesiredJointsPositionProvider desiredJointsProvider;
   private boolean firstPacket = true;

   private final HashMap<OneDoFJoint, Double> previousPosition = new HashMap<OneDoFJoint, Double>();

   private SideDependentList<double[]> armJointAngles, legJointAngles;
   private double[] waistJointAngles;
   private double neckJointAngle;
   

   public JointPositionHighLevelController(MomentumBasedController momentumBasedController, DesiredJointsPositionProvider desiredJointsProvider)
   {
      super(controllerState);

      timeProvider = momentumBasedController.getYoTime();
      this.desiredJointsProvider = desiredJointsProvider;

      trajectoryGenerator = new HashMap<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator>();
      alternativeController = new HashMap<OneDoFJoint, PIDController>();
      useAlternativeController = new HashMap<OneDoFJoint, BooleanYoVariable>();

      fullRobotModel = momentumBasedController.getFullRobotModel();
      trajectoryTimeProvider = new YoVariableDoubleProvider("jointControl_trajectory_time", registry);
      
      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJoints()[i];
         String joinName = joint.getName();

         if (joinName.contains("finger"))
            continue;

         jointsBeenControlled.add(joint);

         OneDoFJointQuinticTrajectoryGenerator generator = new OneDoFJointQuinticTrajectoryGenerator("jointControl_" + joint.getName(), joint,
                                                              trajectoryTimeProvider, registry);
         trajectoryGenerator.put(joint, generator);
         
         PIDController pidController = new PIDController( "jointControl_" + joint.getName() , registry);
         alternativeController.put(joint, pidController);
         
         BooleanYoVariable useAlternative = new BooleanYoVariable( "jointControl_" + joint.getName() + "_enableAlternativePID" , registry);
         useAlternative.set(false);
         useAlternativeController.put(joint, useAlternative ); 
      }
   }

   private void initializeFromPacket(JointAnglesPacket packet)
   {
      initialTrajectoryTime = timeProvider.getDoubleValue();
      System.out.println("initialTrajectoryTime " + initialTrajectoryTime);
      trajectoryTimeProvider.set(packet.getTrajectoryTime());

      System.out.println(" packet.trajectoryTime " + packet.getTrajectoryTime());

      if (armJointAngles == null)
      {
         armJointAngles = new SideDependentList<double[]>();

         for (RobotSide robotSide : RobotSide.values)
         {
            double[] jointAngles = new double[packet.getNumberOfArmJoints(robotSide)];
            armJointAngles.put(robotSide, jointAngles);
         }
      }

      if (this.legJointAngles == null)
      {
         legJointAngles = new SideDependentList<double[]>();

         for (RobotSide robotSide : RobotSide.values)
         {
            double[] jointAngles = new double[packet.getNumberOfLegJoints(robotSide)];
            legJointAngles.put(robotSide, jointAngles);
         }
      }

      if (waistJointAngles == null)
      {
         waistJointAngles = new double[packet.getNumberOfWaistJoints()];
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         setFinalPositionArmsAndLegs(robotSide, packet);
      }

      setFinalPositionSpineJoints(packet);
      setFinalPositionNeckJoint(packet);


      if (firstPacket)
      {
         firstPacket = false;

         for (OneDoFJoint joint : jointsBeenControlled)
         {
            trajectoryGenerator.get(joint).initialize();
            previousPosition.put(joint, joint.getQ());
         }
      }
      else
      {
         for (OneDoFJoint joint : jointsBeenControlled)
         {
            trajectoryGenerator.get(joint).initialize(previousPosition.get(joint), 0.0);
         }
      }
   }
   
   private void setFinalPositionSpineJoints(JointAnglesPacket packet)
   {
      packet.packWaistJointAngle(waistJointAngles);

      SpineJointName[] spineJointNames = fullRobotModel.getRobotSpecificJointNames().getSpineJointNames();
      for(int i=0; i<spineJointNames.length; i++)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getSpineJoint(spineJointNames[i]);
         double desiredPostion = waistJointAngles[i];
         
         //make sure that we do not command an arm joint outside the joint limits
         desiredPostion = MathTools.clipToMinMax(desiredPostion, oneDoFJoint.getJointLimitLower(), oneDoFJoint.getJointLimitUpper());
         
         trajectoryGenerator.get(oneDoFJoint).setFinalPosition(desiredPostion);
      }
      
//    trajectoryGenerator.get(fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH)).setFinalPosition(waistJointAngles[0]);
//    trajectoryGenerator.get(fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL)).setFinalPosition(waistJointAngles[1]);
//    trajectoryGenerator.get(fullRobotModel.getSpineJoint(SpineJointName.SPINE_YAW)).setFinalPosition(waistJointAngles[2]);
   }
   
   private void setFinalPositionNeckJoint(JointAnglesPacket packet)
   {
      neckJointAngle = packet.getNeckJointAngle();
      
      OneDoFJoint neckJoint = fullRobotModel.getNeckJoint(NeckJointName.LOWER_NECK_PITCH);
      if( neckJoint == null) return;
      double desiredNeckJointAngle = MathTools.clipToMinMax(neckJointAngle, neckJoint.getJointLimitLower(), neckJoint.getJointLimitUpper());
      
      OneDoFJointQuinticTrajectoryGenerator trajectory = trajectoryGenerator.get(neckJoint);
      if( trajectory == null) return;
      trajectory.setFinalPosition(desiredNeckJointAngle);
   }
   

   private void setFinalPositionArmsAndLegs(RobotSide robotSide, JointAnglesPacket packet)
   {
      packet.packArmJointAngle(robotSide, armJointAngles.get(robotSide));
      
      ArmJointName[] armJointNames = fullRobotModel.getRobotSpecificJointNames().getArmJointNames();
      for(int i=0; i<armJointNames.length; i++)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getArmJoint(robotSide, armJointNames[i]);
         double desiredPostion = armJointAngles.get(robotSide)[i];
         
         //make sure that we do not command an arm joint outside the joint limits
         desiredPostion = MathTools.clipToMinMax(desiredPostion, oneDoFJoint.getJointLimitLower(), oneDoFJoint.getJointLimitUpper());
                
         trajectoryGenerator.get(oneDoFJoint).setFinalPosition(desiredPostion);
         
         int jointTorqueLimit = 0;
         if( robotSide == RobotSide.LEFT ) {
            jointTorqueLimit = packet.leftArmJointTorqueLimit[i] ;
         }
         else{
            jointTorqueLimit = packet.rightArmJointTorqueLimit[i] ;
         }
         if( jointTorqueLimit > 0 )
         {
            //useAlternativeController.put( oneDoFJoint, useAlternative);
            alternativeController.get(oneDoFJoint).setMaximumOutputLimit( jointTorqueLimit );
         }
         else{
            //useAlternativeController.put( oneDoFJoint, dontUseAlternative);
            alternativeController.get(oneDoFJoint).setMaximumOutputLimit( Double.POSITIVE_INFINITY );
         }
      }
      
      packet.packLegJointAngle(robotSide, legJointAngles.get(robotSide));
      LegJointName[] legJointNames = fullRobotModel.getRobotSpecificJointNames().getLegJointNames();
      for(int i=0; i<legJointNames.length; i++)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getLegJoint(robotSide, legJointNames[i]);
         double desiredPostion = legJointAngles.get(robotSide)[i];
         
         //make sure that we do not command an arm joint outside the joint limits
         desiredPostion = MathTools.clipToMinMax(desiredPostion, oneDoFJoint.getJointLimitLower(), oneDoFJoint.getJointLimitUpper());
         
         trajectoryGenerator.get(oneDoFJoint).setFinalPosition(desiredPostion);
         
         int jointTorqueLimit = 0;
         if( robotSide == RobotSide.LEFT ) {
            jointTorqueLimit = packet.leftLegJointTorqueLimit[i] ;
         }
         else{
            jointTorqueLimit = packet.rightLegJointTorqueLimit[i] ;
         }
         if( jointTorqueLimit > 0 )
         {
          //  useAlternativeController.put( oneDoFJoint, useAlternative);
            alternativeController.get(oneDoFJoint).setMaximumOutputLimit( jointTorqueLimit );
         }
         else{
          //  useAlternativeController.put( oneDoFJoint, dontUseAlternative);
            alternativeController.get(oneDoFJoint).setMaximumOutputLimit( Double.POSITIVE_INFINITY );
         }
      }      
   }

   @Override
   public void doAction()
   {
      if ((desiredJointsProvider != null) && desiredJointsProvider.checkForNewPacket())
      {
         initializeFromPacket(desiredJointsProvider.getNewPacket());
      }

      double time = timeProvider.getDoubleValue() - initialTrajectoryTime;

      for (OneDoFJoint joint : jointsBeenControlled)
      {
         OneDoFJointQuinticTrajectoryGenerator generator = trajectoryGenerator.get(joint);

         if (generator.isDone() == false)
         {
            generator.compute(time);
         }

         if( useAlternativeController.get(joint).getBooleanValue() )
         {
            PIDController controller = alternativeController.get(joint);
            double effort = controller.compute( joint.getQ(), generator.getValue(), joint.getQd(), generator.getVelocity(), 0.003 );
            
            joint.setUnderPositionControl(false);
            joint.setTau( effort );
         }
         else{
            joint.setUnderPositionControl(true);
            joint.setqDesired(generator.getValue());
            joint.setQdDesired(generator.getVelocity());
         }
         
         previousPosition.put(joint, generator.getValue());
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (OneDoFJoint joint : jointsBeenControlled)
      {
         trajectoryGenerator.get(joint).setFinalPosition(joint.getQ());
         trajectoryGenerator.get(joint).initialize(joint.getQ(), joint.getQd());
      }
      
      trajectoryTimeProvider.set(0.1);
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }


   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
   
   @Override
   public void stopExecution()
   {
      //@DAVIDE FREEZEME
      System.out.println( this.getClass().getSimpleName() + " stopExecution. TODO" );
   }


}
