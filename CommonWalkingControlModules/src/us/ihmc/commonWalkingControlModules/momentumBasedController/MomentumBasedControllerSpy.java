package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;

import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class MomentumBasedControllerSpy
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<ExternalWrenchCommand> externalWrenchCommands = new ArrayList<ExternalWrenchCommand>();
   private final ArrayList<OneDoFJointAccelerationCommand> oneDoFJointAccelerationCommands = new ArrayList<OneDoFJointAccelerationCommand>();
   private final ArrayList<DesiredRateOfChangeOfMomentumCommand> desiredRateOfChangeOfMomentumCommands = new ArrayList<DesiredRateOfChangeOfMomentumCommand>();
   private final ArrayList<DesiredSpatialAccelerationCommand> desiredSpatialAccelerationCommands = new ArrayList<DesiredSpatialAccelerationCommand>();

   public MomentumBasedControllerSpy(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   public void setExternalWrenchToCompensateFor(RigidBody rigidBody, Wrench wrench)
   {
      ExternalWrenchCommand externalWrenchCommand = new ExternalWrenchCommand(rigidBody, wrench);
      externalWrenchCommands.add(externalWrenchCommand);
   }

   public void setOneDoFJointAcceleration(OneDoFJoint joint, double desiredAcceleration)
   {
      OneDoFJointAccelerationCommand oneDoFJointAccelerationCommand = new OneDoFJointAccelerationCommand(joint, desiredAcceleration);
      oneDoFJointAccelerationCommands.add(oneDoFJointAccelerationCommand);
   }

   public void setDesiredRateOfChangeOfMomentum(MomentumRateOfChangeData momentumRateOfChangeData)
   {
      DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand = new DesiredRateOfChangeOfMomentumCommand(momentumRateOfChangeData);
      desiredRateOfChangeOfMomentumCommands.add(desiredRateOfChangeOfMomentumCommand);
   }

   public void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData)
   {
      DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand = new DesiredSpatialAccelerationCommand(jacobian, taskspaceConstraintData);
      desiredSpatialAccelerationCommands.add(desiredSpatialAccelerationCommand);
   }

   public void doPrioritaryControl()
   {
      externalWrenchCommands.clear();
      oneDoFJointAccelerationCommands.clear();
      desiredRateOfChangeOfMomentumCommands.clear();
      desiredSpatialAccelerationCommands.clear();
   }

   public void doSecondaryControl()
   {
      StringBuffer stringBuffer = new StringBuffer();
      getCommandsIntoStringBuffer(stringBuffer);

      System.out.println("\n\n***** MomentumBasedControllerSpy: *****\n" + stringBuffer);
   }


   public void getCommandsIntoStringBuffer(StringBuffer stringBuffer)
   {
      for (ExternalWrenchCommand externalWrenchCommand : externalWrenchCommands)
      {
         stringBuffer.append(externalWrenchCommand + "\n");
      }

      for (OneDoFJointAccelerationCommand oneDoFJointAccelerationCommand : oneDoFJointAccelerationCommands)
      {
         stringBuffer.append(oneDoFJointAccelerationCommand + "\n");
      }

      for (DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand : desiredRateOfChangeOfMomentumCommands)
      {
         stringBuffer.append(desiredRateOfChangeOfMomentumCommand + "\n");
      }

      for (DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand : desiredSpatialAccelerationCommands)
      {
         stringBuffer.append(desiredSpatialAccelerationCommand + "\n");
      }

   }

   private class ExternalWrenchCommand
   {
      private final RigidBody rigidBody;
      private final Wrench wrench;
      private final StackTraceElement[] stackTrace;

      public ExternalWrenchCommand(RigidBody rigidBody, Wrench wrench)
      {
         this.rigidBody = rigidBody;
         this.wrench = wrench;

         this.stackTrace = Thread.currentThread().getStackTrace();
      }

      public String toString()
      {
         return getStackInformation(stackTrace) + ", ExternalWrenchCommand: " + rigidBody.getName();
      }
   }



   private class OneDoFJointAccelerationCommand
   {
      private final OneDoFJoint joint;
      private final double desiredAcceleration;
      private final StackTraceElement[] stackTrace;

      public OneDoFJointAccelerationCommand(OneDoFJoint joint, double desiredAcceleration)
      {
         this.joint = joint;
         this.desiredAcceleration = desiredAcceleration;

         this.stackTrace = Thread.currentThread().getStackTrace();
      }

      public String toString()
      {
         return getStackInformation(stackTrace) + ", OneDoFJointAccelerationCommand: " + joint.getName();
      }

   }


   private class DesiredRateOfChangeOfMomentumCommand
   {
      private final MomentumRateOfChangeData momentumRateOfChangeData;
      private final StackTraceElement[] stackTrace;

      public DesiredRateOfChangeOfMomentumCommand(MomentumRateOfChangeData momentumRateOfChangeData)
      {
         this.momentumRateOfChangeData = momentumRateOfChangeData;

         this.stackTrace = Thread.currentThread().getStackTrace();
      }

      public String toString()
      {
         return getStackInformation(stackTrace) + ", DesiredRateOfChangeOfMomentumCommand: MomentumSubspace = "
                + momentumRateOfChangeData.getMomentumSubspace();
      }
   }


   private class DesiredSpatialAccelerationCommand
   {
      private final GeometricJacobian jacobian;
      private final TaskspaceConstraintData taskspaceConstraintData;
      private final StackTraceElement[] stackTrace;

      public DesiredSpatialAccelerationCommand(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData)
      {
         this.jacobian = jacobian;
         this.taskspaceConstraintData = taskspaceConstraintData;

         this.stackTrace = Thread.currentThread().getStackTrace();
      }

      public String toString()
      {
         return getStackInformation(stackTrace) + ", DesiredSpatialAccelerationCommand: GeometricJacobian = " + jacobian + ", taskspaceConstraintData = "
                + taskspaceConstraintData;
      }

   }


   private static String getStackInformation(StackTraceElement[] stackTrace)
   {
      int stackIndexForCallIntoMomentumBasedController = 4;
      StackTraceElement stackTraceElement = stackTrace[stackIndexForCallIntoMomentumBasedController];
      String className = stackTraceElement.getClassName();

      int lastDotIndex = className.lastIndexOf('.');
      className = className.substring(lastDotIndex + 1);

      String methodName = stackTraceElement.getMethodName();
      int lineNumber = stackTrace[0].getLineNumber();

      return className + "." + methodName + "(): Line " + lineNumber;
   }
}
