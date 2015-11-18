package us.ihmc.quadrupedRobotics;

import java.util.ArrayList;

import javax.vecmath.Point3d;

import us.ihmc.SdfLoader.OutputWriter;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedOutputWriter;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.SdfLoader.partNames.JointRole;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedActuatorParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class SimulatedOutputWriterWithControlModeSelection implements OutputWriter
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final SDFPerfectSimulatedOutputWriter outputWriter;
   
   private final ArrayList<PDPositionControllerForOneDoFJoint> positionControllers = new ArrayList<>();

   private final SDFRobot sdfRobot;
   private final SDFFullRobotModel sdfFullRobotModel;
   
   private RobotController robotController;

   private final Point3d comPoint = new Point3d();
   private final YoFramePoint actualCenterOfMassPosition = new YoFramePoint("actualCenterOfMass", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition actualCenterOfMassViz;
   
   public SimulatedOutputWriterWithControlModeSelection(SDFFullRobotModel sdfFullRobotModel, SDFRobot robot, QuadrupedRobotParameters robotParameters, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.sdfRobot = robot;
      this.sdfFullRobotModel = sdfFullRobotModel;
      this.outputWriter = new SDFPerfectSimulatedOutputWriter(robot, sdfFullRobotModel);
      
      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints  = new ArrayList<>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);
      
      createPDControllers(sdfFullRobotModel, robotParameters, oneDegreeOfFreedomJoints);
      
      actualCenterOfMassViz = new YoGraphicPosition("actualCenterOfMass", actualCenterOfMassPosition, 0.04, YoAppearance.DeepPink(), GraphicType.BALL_WITH_CROSS);
      yoGraphicsListRegistry.registerYoGraphic("actualCenterOfMassViz", actualCenterOfMassViz);
      actualCenterOfMassViz.hideGraphicObject();
      parentRegistry.addChild(registry);
   }
   
   public void setHighLevelController(RobotController robotController)
   {
      if(this.robotController != null)
      {
         throw new IllegalArgumentException("Robot controller already registered! Currently, you can only register one");
      }
      this.robotController = robotController;
      registry.addChild(robotController.getYoVariableRegistry());
   }

   @Override
   public void initialize()
   {
      outputWriter.setFullRobotModel(sdfFullRobotModel);
   }
   
   @Override
   public void write()
   {
      for(PDPositionControllerForOneDoFJoint positionController : positionControllers)
      {
         if(positionController.doPositionControl())
         {
            positionController.update();
         }
      }
      
      outputWriter.write();
      
      sdfRobot.computeCenterOfMass(comPoint);
      actualCenterOfMassPosition.set(comPoint);
      actualCenterOfMassViz.update();
   }
   
   private void createPDControllers(SDFFullRobotModel sdfFullRobotModel, QuadrupedRobotParameters robotParameters, ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints)
   {
      QuadrupedJointNameMap jointMap = robotParameters.getJointMap();
      QuadrupedActuatorParameters actuatorParameters = robotParameters.getActuatorParameters();
      for(OneDegreeOfFreedomJoint simulatedJoint : oneDegreeOfFreedomJoints)
      {
         String jointName = simulatedJoint.getName();
         OneDoFJoint oneDoFJoint = sdfFullRobotModel.getOneDoFJointByName(jointName);
         double kp = actuatorParameters.getLegKp();
         double kd = actuatorParameters.getLegKd();
         double maxTorque = actuatorParameters.getLegSoftTorqueLimit();
         
         if(jointMap.getJointRole(jointName) == JointRole.NECK)
         {
            kp = actuatorParameters.getNeckKp();
            kd = actuatorParameters.getNeckKd();
         }
         
         positionControllers.add(new PDPositionControllerForOneDoFJoint(oneDoFJoint, kp, kd, maxTorque));
      }
   }
   
   public class PDPositionControllerForOneDoFJoint
   {
      private final PDController pdController;
      private final YoVariableRegistry pidRegistry;
      private final OneDoFJoint oneDofJoint;
      private final DoubleYoVariable q_d, tau_d;
      private final double maxTorque;
      
      public PDPositionControllerForOneDoFJoint(OneDoFJoint oneDofJoint, double kp, double kd, double maxTorque)
      {
         String name = "pdController_" + oneDofJoint.getName();
         pidRegistry = new YoVariableRegistry(name);
         q_d = new DoubleYoVariable(name + "_q_d", pidRegistry);
         tau_d = new DoubleYoVariable(name + "_tau_d", pidRegistry);
         
         this.maxTorque = maxTorque;
         
         pdController = new PDController(oneDofJoint.getName(), pidRegistry);
         pdController.setProportionalGain(kp);
         pdController.setDerivativeGain(kd);
         registry.addChild(pidRegistry);
         
         this.oneDofJoint = oneDofJoint;
      }
      
      public void update()
      {
         double currentPosition = oneDofJoint.getQ();
         double desiredPosition = oneDofJoint.getqDesired();
         q_d.set(desiredPosition);
         
         double currentRate = oneDofJoint.getQd();
         double desiredRate = oneDofJoint.getQdDesired();
         double desiredTau = pdController.compute(currentPosition, desiredPosition, currentRate, desiredRate);
         
         // Clip the max torque to both better match the servos and also to prevent the simulation from blowing up when dt is fast.
         desiredTau = MathTools.clipToMinMax(desiredTau, maxTorque);
         
         tau_d.set(desiredTau);
         oneDofJoint.setTau(desiredTau);
      }
      
      public boolean doPositionControl()
      {
         return oneDofJoint.isUnderPositionControl();
      }
   }

   @Override
   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      outputWriter.setFullRobotModel(sdfFullRobotModel);
   }
}
