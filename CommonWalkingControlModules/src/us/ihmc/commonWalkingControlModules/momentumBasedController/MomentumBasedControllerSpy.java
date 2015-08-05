package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactableRollingBody;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredPointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredRateOfChangeOfMomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.StackTraceRecorder;
import us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.bipedSupportPolygons.ContactableCylinderBody;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;


public class MomentumBasedControllerSpy
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable printMomentumCommands = new BooleanYoVariable("printMomentumCommands", registry);

   private final IntegerYoVariable numberOfExternalWrenchCommands = new IntegerYoVariable("numExternalWrenchCommands", registry);
   private final IntegerYoVariable numberOfDesiredPointAccelerationCommands = new IntegerYoVariable("numDesiredPointAccelerationCommands", registry);
   private final IntegerYoVariable numberOfDesiredJointAccelerationCommands = new IntegerYoVariable("numDesiredJointAccelerationCommands", registry);
   private final IntegerYoVariable numberOfDesiredRateOfChangeOfMomentumCommands = new IntegerYoVariable("numDesiredRateOfChangeOfMomentumCommands", registry);
   private final IntegerYoVariable numberOfDesiredSpatialAccelerationCommands = new IntegerYoVariable("numDesiredSpatialAccelerationCommands", registry);
   private final IntegerYoVariable numberOfPlaneContactStateCommand = new IntegerYoVariable("numPlaneContactStateCommand", registry);
   private final IntegerYoVariable numberOfRollingContactStateCommand = new IntegerYoVariable("numRollingContactStateCommand", registry);
   private final IntegerYoVariable numberOfCylindricalContactInContactCommand = new IntegerYoVariable("numCylindricalContactInContactCommand", registry);

   private final ArrayList<StackTraceRecorder<ExternalWrenchCommand>> externalWrenchCommands = new ArrayList<StackTraceRecorder<ExternalWrenchCommand>>();
   private final ArrayList<StackTraceRecorder<DesiredPointAccelerationCommand>> desiredPointAccelerationCommands = new ArrayList<StackTraceRecorder<DesiredPointAccelerationCommand>>();
   private final ArrayList<StackTraceRecorder<DesiredJointAccelerationCommand>> desiredJointAccelerationCommands = new ArrayList<StackTraceRecorder<DesiredJointAccelerationCommand>>();
   private final ArrayList<StackTraceRecorder<DesiredRateOfChangeOfMomentumCommand>> desiredRateOfChangeOfMomentumCommands = new ArrayList<StackTraceRecorder<DesiredRateOfChangeOfMomentumCommand>>();
   private final ArrayList<StackTraceRecorder<DesiredSpatialAccelerationCommand>> desiredSpatialAccelerationCommands = new ArrayList<StackTraceRecorder<DesiredSpatialAccelerationCommand>>();
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
      externalWrenchCommands.add(new StackTraceRecorder<ExternalWrenchCommand>(externalWrenchCommand));
   }

   public void setDesiredPointAcceleration(GeometricJacobian rootToEndEffectorJacobian, FramePoint contactPoint, FrameVector desiredAcceleration)
   {
      DesiredPointAccelerationCommand desiredPointAccelerationCommand = new DesiredPointAccelerationCommand(rootToEndEffectorJacobian, contactPoint,
                                                                           desiredAcceleration);
      desiredPointAccelerationCommands.add(new StackTraceRecorder<DesiredPointAccelerationCommand>(desiredPointAccelerationCommand));
   }
   
   public void setDesiredJointAcceleration(OneDoFJoint joint, DenseMatrix64F jointAcceleration)
   {
      DesiredJointAccelerationCommand desiredJointAccelerationCommand = new DesiredJointAccelerationCommand(joint, jointAcceleration);
      desiredJointAccelerationCommands.add(new StackTraceRecorder<DesiredJointAccelerationCommand>(desiredJointAccelerationCommand));
   }

   public void setDesiredRateOfChangeOfMomentum(MomentumRateOfChangeData momentumRateOfChangeData)
   {
      DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand = new DesiredRateOfChangeOfMomentumCommand(momentumRateOfChangeData);
      desiredRateOfChangeOfMomentumCommands.add(new StackTraceRecorder<DesiredRateOfChangeOfMomentumCommand>(desiredRateOfChangeOfMomentumCommand));
   }

   public void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData)
   {
      DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand = new DesiredSpatialAccelerationCommand(jacobian, taskspaceConstraintData);
      desiredSpatialAccelerationCommands.add(new StackTraceRecorder<DesiredSpatialAccelerationCommand>(desiredSpatialAccelerationCommand));
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
      desiredJointAccelerationCommands.clear();
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
      for (StackTraceRecorder<ExternalWrenchCommand> externalWrenchCommand : externalWrenchCommands)
      {
         stringBuffer.append(externalWrenchCommand + "\n");
      }

      for (StackTraceRecorder<DesiredPointAccelerationCommand> desiredPointAccelerationCommand : desiredPointAccelerationCommands)
      {
         stringBuffer.append(desiredPointAccelerationCommand + "\n");
      }

      for (StackTraceRecorder<DesiredJointAccelerationCommand> desiredJointAccelerationCommand : desiredJointAccelerationCommands)
      {
         stringBuffer.append(desiredJointAccelerationCommand + "\n");
      }

      for (StackTraceRecorder<DesiredRateOfChangeOfMomentumCommand> desiredRateOfChangeOfMomentumCommand : desiredRateOfChangeOfMomentumCommands)
      {
         stringBuffer.append(desiredRateOfChangeOfMomentumCommand + "\n");
      }

      for (StackTraceRecorder<DesiredSpatialAccelerationCommand> desiredSpatialAccelerationCommand : desiredSpatialAccelerationCommands)
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
      numberOfDesiredJointAccelerationCommands.set(desiredJointAccelerationCommands.size());
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
      stringBuffer.append(desiredJointAccelerationCommands.size() + " DesiredJointAccelerationCommands\n");
      stringBuffer.append(desiredRateOfChangeOfMomentumCommands.size() + " DesiredRateOfChangeOfMomentumCommands\n");
      stringBuffer.append(desiredSpatialAccelerationCommands.size() + " DesiredSpatialAccelerationCommands\n");

      stringBuffer.append(planeContactStateCommands.size() + " PlaneContactStateCommands\n");
      stringBuffer.append(rollingContactStateCommands.size() + " RollingContactStateCommands\n");
      stringBuffer.append(cylindricalContactInContactCommands.size() + " CylindricalContactInContactCommands\n");
   }

   private class PlaneContactStateCommand
   {
      private final ContactablePlaneBody contactableBody;
      private final StackTraceElement[] stackTrace;

      public PlaneContactStateCommand(ContactablePlaneBody contactableBody, List<FramePoint2d> contactPoints, double coefficientOfFriction,
                                      FrameVector normalContactVector)
      {
         this.contactableBody = contactableBody;

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
      private final StackTraceElement[] stackTrace;

      public RollingContactStateCommand(ContactableRollingBody contactableRollingBody, List<FramePoint2d> contactPoints, double coefficientOfFriction)
      {
         this.contactableRollingBody = contactableRollingBody;

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
