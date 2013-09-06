package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactableCylinderBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactableRollingBody;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class MomentumBasedControllerSpy
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable printMomentumCommands = new BooleanYoVariable("printMomentumCommands", registry);

   private final IntegerYoVariable numberOfExternalWrenchCommands = new IntegerYoVariable("numExternalWrenchCommands", registry);
   private final IntegerYoVariable numberOfDesiredPointAccelerationCommands = new IntegerYoVariable("numDesiredPointAccelerationCommands", registry);
   private final IntegerYoVariable numberOfOneDoFJointAccelerationCommands = new IntegerYoVariable("numOneDoFJointAccelerationCommands", registry);
   private final IntegerYoVariable numberOfDesiredRateOfChangeOfMomentumCommands = new IntegerYoVariable("numDesiredRateOfChangeOfMomentumCommands", registry);
   private final IntegerYoVariable numberOfDesiredSpatialAccelerationCommands = new IntegerYoVariable("numDesiredSpatialAccelerationCommands", registry);
   private final IntegerYoVariable numberOfPlaneContactStateCommand = new IntegerYoVariable("numPlaneContactStateCommand", registry);
   private final IntegerYoVariable numberOfRollingContactStateCommand = new IntegerYoVariable("numRollingContactStateCommand", registry);
   private final IntegerYoVariable numberOfCylindricalContactInContactCommand = new IntegerYoVariable("numCylindricalContactInContactCommand", registry);

   private final ArrayList<ExternalWrenchCommand> externalWrenchCommands = new ArrayList<ExternalWrenchCommand>();
   private final ArrayList<DesiredPointAccelerationCommand> desiredPointAccelerationCommands = new ArrayList<DesiredPointAccelerationCommand>();
   private final ArrayList<OneDoFJointAccelerationCommand> oneDoFJointAccelerationCommands = new ArrayList<OneDoFJointAccelerationCommand>();
   private final ArrayList<DesiredRateOfChangeOfMomentumCommand> desiredRateOfChangeOfMomentumCommands = new ArrayList<DesiredRateOfChangeOfMomentumCommand>();
   private final ArrayList<DesiredSpatialAccelerationCommand> desiredSpatialAccelerationCommands = new ArrayList<DesiredSpatialAccelerationCommand>();
   private final ArrayList<PlaneContactStateCommand> planeContactStateCommands = new ArrayList<PlaneContactStateCommand>();
   private final ArrayList<RollingContactStateCommand> rollingContactStateCommands = new ArrayList<RollingContactStateCommand>();
   private final ArrayList<CylindricalContactInContactCommand> cylindricalContactInContactCommands = new ArrayList<CylindricalContactInContactCommand>();

   public MomentumBasedControllerSpy(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   public void setExternalWrenchToCompensateFor(RigidBody rigidBody, Wrench wrench)
   {
      ExternalWrenchCommand externalWrenchCommand = new ExternalWrenchCommand(rigidBody, wrench);
      externalWrenchCommands.add(externalWrenchCommand);
   }

   public void setDesiredPointAcceleration(GeometricJacobian rootToEndEffectorJacobian, FramePoint contactPoint, FrameVector desiredAcceleration)
   {
      DesiredPointAccelerationCommand desiredPointAccelerationCommand = new DesiredPointAccelerationCommand(rootToEndEffectorJacobian, contactPoint,
                                                                           desiredAcceleration);
      desiredPointAccelerationCommands.add(desiredPointAccelerationCommand);
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

   public void setPlaneContactState(ContactablePlaneBody contactableBody, List<FramePoint2d> contactPoints, double coefficientOfFriction,
                                    FrameVector normalContactVector)
   {
      PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand(contactableBody, contactPoints, coefficientOfFriction,
                                                             normalContactVector);
      planeContactStateCommands.add(planeContactStateCommand);
   }

   public void setRollingContactState(ContactableRollingBody contactableRollingBody, List<FramePoint2d> contactPoints, double coefficientOfFriction)
   {
      RollingContactStateCommand rollingContactStateCommand = new RollingContactStateCommand(contactableRollingBody, contactPoints, coefficientOfFriction);
      rollingContactStateCommands.add(rollingContactStateCommand);

   }

   public void setCylindricalContactInContact(ContactableCylinderBody contactableCylinderBody, boolean setInContact)
   {
      CylindricalContactInContactCommand cylindricalContactInContactCommand = new CylindricalContactInContactCommand(contactableCylinderBody, setInContact);
      cylindricalContactInContactCommands.add(cylindricalContactInContactCommand);
   }

   public void doPrioritaryControl()
   {
      externalWrenchCommands.clear();
      desiredPointAccelerationCommands.clear();
      oneDoFJointAccelerationCommands.clear();
      desiredRateOfChangeOfMomentumCommands.clear();
      desiredSpatialAccelerationCommands.clear();

      planeContactStateCommands.clear();
      rollingContactStateCommands.clear();
      cylindricalContactInContactCommands.clear();
   }

   public void doSecondaryControl()
   {
      setYoVariables();

      if (printMomentumCommands.getBooleanValue())
      {
         printMomentumCommands(System.out);

         printMomentumCommands.set(false);
      }

   }

   public void printMomentumCommands(PrintStream printStream)
   {
      StringBuffer stringBuffer = new StringBuffer();
      getCommandsIntoStringBufferBrief(stringBuffer);
      printStream.println("\n\n***** MomentumBasedControllerSpy: *****\n" + stringBuffer);

      stringBuffer = new StringBuffer();

      getCommandsIntoStringBufferVerbose(stringBuffer);
      printStream.println("\n\n***** MomentumBasedControllerSpy: *****\n" + stringBuffer);
   }


   private void getCommandsIntoStringBufferVerbose(StringBuffer stringBuffer)
   {
      for (ExternalWrenchCommand externalWrenchCommand : externalWrenchCommands)
      {
         stringBuffer.append(externalWrenchCommand + "\n");
      }

      for (DesiredPointAccelerationCommand desiredPointAccelerationCommand : desiredPointAccelerationCommands)
      {
         stringBuffer.append(desiredPointAccelerationCommand + "\n");
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

      for (PlaneContactStateCommand planeContactStateCommand : planeContactStateCommands)
      {
         stringBuffer.append(planeContactStateCommand + "\n");
      }
      
      for (RollingContactStateCommand rollingContactStateCommand : rollingContactStateCommands)
      {
         stringBuffer.append(rollingContactStateCommand + "\n");
      }
      
      for (CylindricalContactInContactCommand cylindricalContactInContactCommand : cylindricalContactInContactCommands)
      {
         stringBuffer.append(cylindricalContactInContactCommand + "\n");
      }

   }

   private void setYoVariables()
   {
      numberOfExternalWrenchCommands.set(externalWrenchCommands.size());
      numberOfDesiredPointAccelerationCommands.set(desiredPointAccelerationCommands.size());
      numberOfOneDoFJointAccelerationCommands.set(oneDoFJointAccelerationCommands.size());
      numberOfDesiredRateOfChangeOfMomentumCommands.set(desiredRateOfChangeOfMomentumCommands.size());
      numberOfDesiredSpatialAccelerationCommands.set(desiredSpatialAccelerationCommands.size());

      numberOfPlaneContactStateCommand.set(planeContactStateCommands.size());
      numberOfRollingContactStateCommand.set(rollingContactStateCommands.size());
      numberOfCylindricalContactInContactCommand.set(cylindricalContactInContactCommands.size());
   }

   private void getCommandsIntoStringBufferBrief(StringBuffer stringBuffer)
   {
      stringBuffer.append(externalWrenchCommands.size() + " ExternalWrenchCommands\n");
      stringBuffer.append(desiredPointAccelerationCommands.size() + " DesiredPointAccelerationCommands\n");
      stringBuffer.append(oneDoFJointAccelerationCommands.size() + " OneDoFJointAccelerationCommands\n");
      stringBuffer.append(desiredRateOfChangeOfMomentumCommands.size() + " DesiredRateOfChangeOfMomentumCommands\n");
      stringBuffer.append(desiredSpatialAccelerationCommands.size() + " DesiredSpatialAccelerationCommands\n");

      stringBuffer.append(planeContactStateCommands.size() + " PlaneContactStateCommands\n");
      stringBuffer.append(rollingContactStateCommands.size() + " RollingContactStateCommands\n");
      stringBuffer.append(cylindricalContactInContactCommands.size() + " CylindricalContactInContactCommands\n");
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
         return getStackInformation(stackTrace) + ", ExternalWrenchCommand: " + wrench;
      }
   }


   private class DesiredPointAccelerationCommand
   {
      private final GeometricJacobian rootToEndEffectorJacobian;
      private final FramePoint contactPoint;
      private final FrameVector desiredAcceleration;

      private final StackTraceElement[] stackTrace;

      public DesiredPointAccelerationCommand(GeometricJacobian rootToEndEffectorJacobian, FramePoint contactPoint, FrameVector desiredAcceleration)
      {
         this.rootToEndEffectorJacobian = rootToEndEffectorJacobian;
         this.contactPoint = contactPoint;
         this.desiredAcceleration = desiredAcceleration;

         this.stackTrace = Thread.currentThread().getStackTrace();
      }

      public String toString()
      {
         return getStackInformation(stackTrace) + ", DesiredPointAccelerationCommand: rootToEndEffectorJacobian = " + rootToEndEffectorJacobian;
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
         return getStackInformation(stackTrace) + ", DesiredSpatialAccelerationCommand: GeometricJacobian = " + jacobian.getShortInfo() + ", taskspaceConstraintData = "
                + taskspaceConstraintData;
      }

   }

   private class PlaneContactStateCommand
   {
      private final ContactablePlaneBody contactableBody;
      private final List<FramePoint2d> contactPoints;
      private final double coefficientOfFriction;
      private final FrameVector normalContactVector;
      private final StackTraceElement[] stackTrace;

      public PlaneContactStateCommand(ContactablePlaneBody contactableBody, List<FramePoint2d> contactPoints, double coefficientOfFriction,
                                      FrameVector normalContactVector)
      {
         this.contactableBody = contactableBody;
         this.contactPoints = contactPoints;
         this.coefficientOfFriction = coefficientOfFriction;
         this.normalContactVector = normalContactVector;

         this.stackTrace = Thread.currentThread().getStackTrace();
      }

      public String toString()
      {
         return getStackInformation(stackTrace) + ", PlaneContactStateCommand: contactableBody = " + contactableBody.getName();
      }
   }


   private class RollingContactStateCommand
   {
      private final ContactableRollingBody contactableRollingBody;
      private final List<FramePoint2d> contactPoints;
      private final double coefficientOfFriction;
      private final StackTraceElement[] stackTrace;

      public RollingContactStateCommand(ContactableRollingBody contactableRollingBody, List<FramePoint2d> contactPoints, double coefficientOfFriction)
      {
         this.contactableRollingBody = contactableRollingBody;
         this.contactPoints = contactPoints;
         this.coefficientOfFriction = coefficientOfFriction;

         this.stackTrace = Thread.currentThread().getStackTrace();
      }

      public String toString()
      {
         return getStackInformation(stackTrace) + ", RollingContactStateCommand: contactableRollingBody = " + contactableRollingBody.getName();
      }
   }


   private class CylindricalContactInContactCommand
   {
      private final ContactableCylinderBody contactableCylinderBody;
      private final boolean setInContact;
      private final StackTraceElement[] stackTrace;

      public CylindricalContactInContactCommand(ContactableCylinderBody contactableCylinderBody, boolean setInContact)
      {
         this.contactableCylinderBody = contactableCylinderBody;
         this.setInContact = setInContact;

         this.stackTrace = Thread.currentThread().getStackTrace();
      }

      public String toString()
      {
         return getStackInformation(stackTrace) + ", CylindricalContactInContactCommand: contactableCylinderBody = " + contactableCylinderBody.getName()
                + " setInContact = " + setInContact;
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
      int lineNumber = stackTrace[stackIndexForCallIntoMomentumBasedController].getLineNumber();

      return "+++ " + className + "." + methodName + "(): Line " + lineNumber;
   }

}
