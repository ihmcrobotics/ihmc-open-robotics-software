package us.ihmc.steppr.hardware.state;

import java.util.EnumMap;

import javax.vecmath.Quat4d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.acsell.parameters.BonoRobotModel;
import us.ihmc.commonWalkingControlModules.visualizer.RobotVisualizer;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class StepprStateCommunicator implements StepprStateProcessor
{

   public static void main(String[] args)
   {
      YoVariableRegistry registry = new YoVariableRegistry("viz");
      StepprState state = new StepprState(registry);
      YoVariableServer variableServer = new YoVariableServer(5555, 0.01);

      StepprStateCommunicator communicator = new StepprStateCommunicator(state, variableServer, registry);

      PriorityParameters priority = new PriorityParameters(PriorityParameters.getMaximumPriority());
      UDPStepprStateReader reader = new UDPStepprStateReader(priority, state, communicator);
      
      variableServer.start();
      reader.start();
      
      ThreadTools.sleepForever();
   }

   private final RobotVisualizer visualizer;
   private final StepprState state;
   private final SixDoFJoint rootJoint;
   private final Quat4d rotation = new Quat4d();
   private final EnumMap<StepprJoint, OneDoFJoint> jointMap = new EnumMap<>(StepprJoint.class);

   public StepprStateCommunicator(StepprState state, RobotVisualizer visualizer, YoVariableRegistry registry)
   {
      this.state = state;
      this.visualizer = visualizer;

      BonoRobotModel robotModel = new BonoRobotModel(true, true);
      SDFFullRobotModel fullRobotModel = robotModel.createFullRobotModel();
      rootJoint = fullRobotModel.getRootJoint();

      for (StepprJoint joint : StepprJoint.values)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(joint.getSdfName());
         if (oneDoFJoint == null)
         {
            throw new RuntimeException("Unknown joint: " + joint.getSdfName());
         }
         jointMap.put(joint, oneDoFJoint);
      }

      if (visualizer != null)
      {
         visualizer.setMainRegistry(registry, fullRobotModel, null);
      }

   }

   @Override
   public void process(long timestamp)
   {
      for (StepprJoint joint : StepprJoint.values)
      {
         StepprJointState jointState = state.getJointState(joint);
         OneDoFJoint oneDoFJoint = jointMap.get(joint);

         oneDoFJoint.setQ(jointState.getQ());
         oneDoFJoint.setQd(jointState.getQd());
      }

      StepprXSensState xSensState = state.getXSensState();
      xSensState.getQuaternion(rotation);
      rootJoint.setRotation(rotation);

      rootJoint.updateFramesRecursively();

      if (visualizer != null)
      {
         visualizer.update(timestamp, 0);
      }
   }

   @Override
   public void initialize(long timestamp)
   {

   }
}
