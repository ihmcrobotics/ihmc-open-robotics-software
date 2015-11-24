package us.ihmc.exampleSimulations.fourBarLinkage;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class FourBarLinkageController implements RobotController
{
   private FourBarLinkageRobot robot;
   private String name;
   private YoVariableRegistry registry = new YoVariableRegistry("controllerRegistry");
//   private final YoGraphicsList yoGraphicsList = new YoGraphicsList(name);
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private PIDController controllerActuator;
   private DoubleYoVariable desiredDistance;
   
   private final YoFramePoint actuatorAnchor1Pos = new YoFramePoint("yoDesiredPositionSwingFoot", worldFrame, registry);
   
   private final YoFramePoint yoPositionFix;
   private final YoFrameVector yoVelocityFix;
   private final YoFrameVector yoTauFix;
   private final DoubleYoVariable fix_kp = new DoubleYoVariable("fixGain", registry);
   private final DoubleYoVariable fix_kd = new DoubleYoVariable("fixDamping", registry);
   private final DoubleYoVariable positionErrorMagnitude = new DoubleYoVariable("positionErrorMagnitude", registry);
   
   private Point3d posEfpFix = new Point3d();
   private Point3d posEfpFixDesired;
   private Point3d velEfpFix = new Point3d();
   private Point3d velEfpFixDesired = new Point3d(0.0,0.0,0.0);
   private Vector3d pTau = new Vector3d();
   private Vector3d dTau = new Vector3d();
   private Vector3d totalTauFix = new Vector3d();
   
   public FourBarLinkageController(FourBarLinkageRobot robot, String name, FourBarLinkageParameters fourBarLinkageParameters)
   {
      this.robot = robot;
      this.name = name;
      
      ExternalForcePoint efpFix = robot.getEfpFixInWorld();
      yoPositionFix = efpFix.getYoPosition();
      posEfpFixDesired = new Point3d(fourBarLinkageParameters.linkageLength_2, 0.0, 0.0);
      yoVelocityFix = efpFix.getYoVelocity();
      yoTauFix = efpFix.getYoForce();  
      
      initializeControls();
//      initializeVisualizers();
   }
   
   public void initializeControls()
   {
      controllerActuator = new PIDController("actuatorController" + name, registry);
      controllerActuator.setProportionalGain(100.0);
      controllerActuator.setDerivativeGain(10.0);
      
      desiredDistance = new DoubleYoVariable("desiredPosition", registry);
      desiredDistance.set(0.3);
      
      fix_kp.set(1000.0);
      fix_kd.set(10.0);
   }
   
//   public void initializeVisualizers()
//   {
//      YoGraphicPosition actuatorAnchor1Viz= new YoGraphicPosition("actuatorAnchor1", actuatorAnchor1Pos, 0.06, YoAppearance.CornflowerBlue());
//      yoGraphicsList.add(actuatorAnchor1Viz);
//   }
   
   
   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
      // Fix Joint 2 with respect to world
      
      yoPositionFix.get(posEfpFix);   
      pTau.sub(posEfpFix, posEfpFixDesired);
      positionErrorMagnitude.set(pTau.length());
      pTau.scale(-fix_kp.getDoubleValue());

      yoVelocityFix.get(velEfpFix);
      dTau.sub(velEfpFix, velEfpFixDesired);
      dTau.scale(-fix_kd.getDoubleValue());

      totalTauFix.add(pTau, dTau); 
      yoTauFix.set(totalTauFix);
   }
   
//   public YoGraphicsList getYoGraphicsList()
//   {
//      return yoGraphicsList;
//   }
}
