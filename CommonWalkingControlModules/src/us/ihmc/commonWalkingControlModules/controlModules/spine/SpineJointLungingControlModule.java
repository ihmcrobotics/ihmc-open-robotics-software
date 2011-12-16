package us.ihmc.commonWalkingControlModules.controlModules.spine;

import java.util.ArrayList;
import java.util.EnumMap;


import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SpineLungingControlModule;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineTorques;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.utilities.containers.ContainerTools;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.Line2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.PIDController;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector2d;

public class SpineJointLungingControlModule implements SpineLungingControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SpineJointLungingControlModule");
   
   public ArrayList<DoubleYoVariable> spinePitchErrorList = new ArrayList<DoubleYoVariable>();

   private final EnumMap<SpineJointName, DoubleYoVariable> desiredAngles = ContainerTools.createEnumMap(SpineJointName.class);
   private final EnumMap<SpineJointName, PIDController> spineControllers = ContainerTools.createEnumMap(SpineJointName.class);
   private final PIDController spineTorqueController = new PIDController("spingeTorqueCtrl", registry);
   private final PIDController spineCmpWrenchControllerX = new PIDController("spineCmPWrenchXCtrl", registry);
   private final PIDController spineCmpWrenchControllerY = new PIDController("spineCmPWrenchYCtrl", registry);
   
   private final ArrayList<PIDController> spineCmpWrenchControllers = new ArrayList<PIDController>();
   
   private final YoFrameVector wrenchOnPelvisAngular = new YoFrameVector("wrenchOnPelvisAngular", "", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector wrenchOnPelvisLinear = new YoFrameVector("wrenchOnPelvislinear", "", ReferenceFrame.getWorldFrame(), registry);

   private final YoFrameVector additionalWrenchAngularPart = new YoFrameVector("additionalWrenchAngularPart", "", ReferenceFrame.getWorldFrame(), registry);
//   private final EnumMap<SpineJointName, DoubleYoVariable> actualAngles;
//   private final EnumMap<SpineJointName, DoubleYoVariable> actualAngleVelocities;
   private YoFramePoint2d desiredCMP;
   private YoFrameVector2d cmpError;
   private YoFrameVector2d cmpWrenchFeedback;

   private final ProcessedSensorsInterface processedSensors;
   private final double controlDT;
   
   private final EnumMap<SpineJointName, PIDController> spineJointIDQddControllers = ContainerTools.createEnumMap(SpineJointName.class);
   private InverseDynamicsCalculator spineJointIDCalc;
   private RigidBody pelvisRigidBody;
   private ArrayList<RevoluteJoint> spineRevoluteJointList;
   
   private FrameVector desiredTorqueBetweenPelvisAndChest;
   private SpineTorques spineTorques = new SpineTorques();

   private final RigidBody chest;

   private Wrench externalWrench;

   private final CommonWalkingReferenceFrames referenceFrames;
 
   public SpineJointLungingControlModule(ProcessedSensorsInterface processedSensors, double controlDT, YoVariableRegistry parentRegistry,
         InverseDynamicsCalculator spineJointIDCalc, RigidBody chest, ArrayList<RevoluteJoint> spineRevoluteJointList,
         CommonWalkingReferenceFrames referenceFrames)
   {
      this.spineJointIDCalc = spineJointIDCalc;
//      this.pelvisRigidBody = pelvisRigidBody;
      this.pelvisRigidBody = processedSensors.getFullRobotModel().getPelvis();
      this.spineRevoluteJointList = spineRevoluteJointList;
      this.processedSensors = processedSensors;
      this.controlDT = controlDT;
      this.chest = chest;
      this.referenceFrames = referenceFrames;
      parentRegistry.addChild(registry);
      
      populateYoVariables();
      populateControllers();
      initializeVariables();
      setDesireds();
      setGains();
   }
   
   private void initializeVariables()
   {
      ReferenceFrame pelvisFrame = pelvisRigidBody.getBodyFixedFrame();
      externalWrench = new Wrench(pelvisFrame, pelvisFrame);
   }

   /**
    * Old. Do not use.
    */
   public void doSpineControl(SpineTorques spineTorquesToPack)
   {
      for (SpineJointName spineJointName : SpineJointName.values())
      {
         PIDController pidController = spineControllers.get(spineJointName);
         
         double desiredPosition = desiredAngles.get(spineJointName).getDoubleValue();
         double desiredVelocity = 0.0;

         double actualPosition = processedSensors.getSpineJointPosition(spineJointName); // actualAngles.get(spineJointName).getDoubleValue();
         double actualVelocity = processedSensors.getSpineJointVelocity(spineJointName); //actualAngleVelocities.get(spineJointName).getDoubleValue();

         double torque = pidController.compute(actualPosition, desiredPosition, actualVelocity, desiredVelocity, controlDT);
         spineTorques.setTorque(spineJointName, torque);
      }
   }

   /**
    * Sets the pitch and roll spine torques corresponding to the desired deltaCMP 
    * @param deltaCMP: The normalized vector from the desired ICP to the current ICP (in the x-y plane), scaled by the CMP radius corresponding to the maximum hip torque
    * @param spineTorquesToPack: The spine pitch and roll torques corresponding to a 90 degree rotation of deltaCMP about the z-axis
    */
   public void setPitchRollSpineTorquesFromDeltaCMP(Vector2d deltaCMP)
   {
      double mass = processedSensors.getTotalMass();
      double gravity = processedSensors.getGravityInWorldFrame().getZ();

//      this.spineTorques.setTorque(SpineJointName.SPINE_PITCH, mass * gravity * deltaCMP.getX());
//      this.spineTorques.setTorque(SpineJointName.SPINE_ROLL, mass * gravity * - deltaCMP.getY());
      this.spineTorques.setTorque(SpineJointName.SPINE_ROLL, mass * gravity * - deltaCMP.getX());
      this.spineTorques.setTorque(SpineJointName.SPINE_PITCH, mass * gravity * - deltaCMP.getY());
   }
   

   private void populateYoVariables()
   {
      for (SpineJointName spineJointName : SpineJointName.values())
      {
         String name = "desired" + spineJointName.getCamelCaseNameForMiddleOfExpression();
         DoubleYoVariable variable = new DoubleYoVariable(name, registry);
         desiredAngles.put(spineJointName, variable);
         
         spinePitchErrorList.add(new DoubleYoVariable(spineJointName + "Error", registry));
      }
      
      this.cmpError = new YoFrameVector2d("cmpError", "", referenceFrames.getMidFeetZUpFrame(), registry);
      this.cmpWrenchFeedback = new YoFrameVector2d("cmpWrenchFeedBack", "", referenceFrames.getMidFeetZUpFrame(), registry); // frame does not matter
   }

   private void populateControllers()
   {
      for (SpineJointName spineJointName : SpineJointName.values())
      {
         spineControllers.put(spineJointName, new PIDController(spineJointName.getCamelCaseNameForStartOfExpression() + "ctrl", registry));
         spineJointIDQddControllers.put(spineJointName, new PIDController(spineJointName.getCamelCaseNameForStartOfExpression() + "qddDesired" + "ctrl", registry));            
      }
      spineCmpWrenchControllers.add(spineCmpWrenchControllerX);
      spineCmpWrenchControllers.add(spineCmpWrenchControllerY);
   }

   private void setDesireds()
   {
      /*
       * 100610 pdn: I tried setting this to 0.1 immediately and the robot fell
       * I could start off with it 0.05 and once it got walking, change it to 0.1
       */
      desiredAngles.get(SpineJointName.SPINE_PITCH).set(0.0);
   }

   public void setGains()
   {
      spineControllers.get(SpineJointName.SPINE_YAW).setProportionalGain(3000.0);
      spineControllers.get(SpineJointName.SPINE_PITCH).setProportionalGain(3000.0);
      spineControllers.get(SpineJointName.SPINE_ROLL).setProportionalGain(3000.0);

      spineControllers.get(SpineJointName.SPINE_YAW).setDerivativeGain(200.0);
      spineControllers.get(SpineJointName.SPINE_PITCH).setDerivativeGain(200.0);
      spineControllers.get(SpineJointName.SPINE_ROLL).setDerivativeGain(200.0);
      
      
      spineJointIDQddControllers.get(SpineJointName.SPINE_YAW).setProportionalGain(3000.0);
      spineJointIDQddControllers.get(SpineJointName.SPINE_PITCH).setProportionalGain(700.0);
      spineJointIDQddControllers.get(SpineJointName.SPINE_ROLL).setProportionalGain(10000.0);

      spineJointIDQddControllers.get(SpineJointName.SPINE_YAW).setDerivativeGain(200.0);
      spineJointIDQddControllers.get(SpineJointName.SPINE_PITCH).setDerivativeGain(100.0);
      spineJointIDQddControllers.get(SpineJointName.SPINE_ROLL).setDerivativeGain(1000.0);
      
      spineTorqueController.setProportionalGain(1.0);
      
      for (int index = 0; index < spineCmpWrenchControllers.size(); index ++)
      {
         spineCmpWrenchControllers.get(index).setProportionalGain(10.0);
         spineCmpWrenchControllers.get(index).setDerivativeGain(1.0);
      }

   }
   
   public void doMaintainDesiredChestOrientation()
   {
      spineTorques.setTorquesToZero();
      
      setDesiredAccelerationOnSpineJoints();
      
      spineJointIDCalc.compute();
      
      setSpineTorquesFromSpineJoints();
   }
   
   public void getWrenchByUpperBody(Wrench upperBodyWrenchToPack)
   {
      processedSensors.getFullRobotModel().getRootJoint().packWrench(upperBodyWrenchToPack);
      upperBodyWrenchToPack.changeBodyFrameAttachedToSameBody(referenceFrames.getPelvisFrame());
      upperBodyWrenchToPack.changeFrame(referenceFrames.getPelvisFrame());
   }
   
   public void setGainsToZero(ArrayList<SpineJointName> spineJointsWithZeroGain)
   {
      for (int index = 0; index < spineJointsWithZeroGain.size(); index ++)
      {
         SpineJointName spineJointName = spineJointsWithZeroGain.get(index);
         spineJointIDQddControllers.get(spineJointName).setProportionalGain(0.0);
         spineJointIDQddControllers.get(spineJointName).setDerivativeGain(0.0);
      }
   }
   
   public void setWrench(Wrench wrenchOnPelvis)
   {
      // to prevent wrong frame stuff
      externalWrench.setAngularPart(wrenchOnPelvis.getAngularPartCopy());
      
      setExternalWrench();
   }

   private void setExternalWrench()
   {
      spineJointIDCalc.setExternalWrench(pelvisRigidBody, externalWrench);
      
      this.wrenchOnPelvisAngular.set(externalWrench.getAngularPartCopy());
      this.wrenchOnPelvisLinear.set(externalWrench.getLinearPartCopy());
   }

   public void getSpineTorques(SpineTorques spineTorquesToPack)
   {
      spineTorquesToPack.setTorques(this.spineTorques.getTorquesCopy());
   }

   public void scaleGainsBasedOnLungeAxis(Vector2d lungeAxis)
   {
      lungeAxis.absolute();
      lungeAxis.normalize();

      // x
      double scaleFactor = lungeAxis.getX(); 
      SpineJointName spineJointName = SpineJointName.SPINE_PITCH;
      scalePDGainsForIDController(scaleFactor, spineJointName);

      // y
      scaleFactor = lungeAxis.getY(); 
      spineJointName = SpineJointName.SPINE_ROLL;
      scalePDGainsForIDController(scaleFactor, spineJointName);
   }

   private void scalePDGainsForIDController(double scaleFactor, SpineJointName spineJointName)
   {
      double proportionalGain = spineJointIDQddControllers.get(spineJointName).getProportionalGain();
      spineJointIDQddControllers.get(spineJointName).setProportionalGain(proportionalGain * scaleFactor);
      
      double derivativeGain = spineJointIDQddControllers.get(spineJointName).getDerivativeGain();
      spineJointIDQddControllers.get(spineJointName).setDerivativeGain(derivativeGain * scaleFactor);
   }
   
   private void setDesiredAccelerationOnSpineJoints()
   {
      for (SpineJointName spineJointName : SpineJointName.values())
      {
         RevoluteJoint spineRevoluteJoint = spineRevoluteJointList.get(spineJointName.ordinal());
         
         double actualPosition = processedSensors.getSpineJointPosition(spineJointName);
         double actualVelocity = processedSensors.getSpineJointVelocity(spineJointName);
         
         double desiredPosition = desiredAngles.get(spineJointName).getDoubleValue();
         double desiredVelocity = 0.0;
         
         spinePitchErrorList.get(spineJointName.ordinal()).set(0.0 - actualPosition);
         double qddDesired = spineJointIDQddControllers.get(spineJointName).compute(actualPosition, desiredPosition, actualVelocity, desiredVelocity, controlDT);
         spineRevoluteJoint.setQddDesired(qddDesired);
      }
   }


   private void setSpineTorquesFromSpineJoints()
   {
      for (SpineJointName spineJointName : SpineJointName.values())
      {
         RevoluteJoint spineRevoluteJoint = spineRevoluteJointList.get(spineJointName.ordinal());
         spineTorques.setTorque(spineJointName, spineRevoluteJoint.getTau());   
      }
   }

   public void setHipXYTorque(Vector3d desiredTorqueVector)
   {
      spineTorques.setTorque(SpineJointName.SPINE_PITCH, desiredTorqueVector.getY());
      spineTorques.setTorque(SpineJointName.SPINE_ROLL, desiredTorqueVector.getX());
      spineTorques.setTorque(SpineJointName.SPINE_YAW, desiredTorqueVector.getZ());
   }
   
   public void doCMPControl(FramePoint2d desiredCMP, FrameVector2d lungeAxis)
   {
      // frame in which everything is expressed
      ReferenceFrame expressedInFrame = referenceFrames.getMidFeetZUpFrame();
      // check if input is in this frame
      desiredCMP.checkReferenceFrameMatch(expressedInFrame);

      // storage
      FramePoint2d currentCMP = this.processedSensors.getCentroidalMomentPivotInFrame(expressedInFrame).toFramePoint2d();

      // error
      this.cmpError.set(desiredCMP);
      this.cmpError.sub(currentCMP);

//      cmpWrenchFeedback.setX(this.spineCmpWrenchController.compute(currentCmpDistance.getDoubleValue(), desiredCmpDistance.getDoubleValue(), 0.0, 0.0, controlDT));
      cmpWrenchFeedback.setX( this.spineCmpWrenchControllers.get(0).compute(currentCMP.getX(), desiredCMP.getX(), 0.0, 0.0, controlDT));
      cmpWrenchFeedback.setY( this.spineCmpWrenchControllers.get(1).compute(currentCMP.getY(), desiredCMP.getY(), 0.0, 0.0, controlDT));
      
//    setting torque      
//    *********************
      // 100 for x is too big
      Vector3d extraTorque = new Vector3d( 100.0 * cmpWrenchFeedback.getY(), - 100.0 * cmpWrenchFeedback.getX(), 0.0);
//      Vector3d extraTorque = new Vector3d( - 100.0 * lungeAxis.getY() * cmpWrenchFeedback.getY(), 100.0 * lungeAxis.getX() * cmpWrenchFeedback.getX(), 0.0);
      // adjust extra torque based on lunge axis (now scales with same magnitude along different axes)
      
      Vector3d lungingTorque  = new Vector3d();
      lungeAxis.changeFrame(ReferenceFrame.getWorldFrame());
      lungingTorque.set(lungeAxis.getX(), lungeAxis.getY(), 0.0);
      lungingTorque.scale( - 150.0);
      lungingTorque.add(extraTorque);
      this.setHipXYTorque(lungingTorque);
//    *********************
      
      
//      setting wrench
//      *********************
//      Vector3d wrenchAngularPart = new Vector3d(lungeAxis.getX(), lungeAxis.getY(), 0.0);
//      wrenchAngularPart.scale(150.0);
//      externalWrench.setLinearPart(new Vector3d());
//      externalWrench.setAngularPart(wrenchAngularPart);
//      this.externalWrench.scale( 1.0 - cmpWrenchFeedback.getX());
//
//      this.setExternalWrench();
//      this.doMaintainDesiredChestOrientation();
//      *********************
   }
   
   
   /**
    * only works well for lunging over x or y axis
    */
   public void doConstantTorqueAroundLungeAxis(FrameVector2d lungeAxis, double constantTorque)
   {
      Vector3d wrenchAngularPart = new Vector3d(lungeAxis.getX(), lungeAxis.getY(), 0.0);
      wrenchAngularPart.scale(constantTorque);
      
      Wrench wrench = new Wrench(externalWrench);
      wrench.setLinearPart(new Vector3d());
      wrench.setAngularPart(wrenchAngularPart);
      this.setWrench(wrench);
      this.doMaintainDesiredChestOrientation();

      Vector3d desiredTorque = new Vector3d(wrenchAngularPart);
      desiredTorque.negate();

      Vector3d currentTorque = new Vector3d(spineTorques.getTorque(SpineJointName.SPINE_ROLL), spineTorques.getTorque(SpineJointName.SPINE_PITCH), spineTorques.getTorque(SpineJointName.SPINE_YAW));
      double angularAdditionX = - spineTorqueController.compute(currentTorque.getX(), desiredTorque.getX(), 0.0, 0.0, controlDT);
      double angularAdditionY = - spineTorqueController.compute(currentTorque.getY(), desiredTorque.getY(), 0.0, 0.0, controlDT);

      //      sideways = > +
      //      backwards = > -

      if (lungeAxis.getX() < lungeAxis.getY())
      {
         angularAdditionY = - angularAdditionY;
      }

      additionalWrenchAngularPart.set(angularAdditionX, angularAdditionY, 0.0);

      wrenchAngularPart.add( additionalWrenchAngularPart.getFrameVectorCopy().getVector());
      wrench.setAngularPart(wrenchAngularPart);
      this.setWrench(wrench);
      this.doMaintainDesiredChestOrientation();
   }
}