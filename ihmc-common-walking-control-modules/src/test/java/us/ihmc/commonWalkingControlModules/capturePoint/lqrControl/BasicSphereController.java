package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class BasicSphereController implements RobotController
{
   private enum SphereControllerEnum {BASIC}

   private static final SphereControllerEnum controllerType = SphereControllerEnum.BASIC;

   private final YoVariableRegistry registry = new YoVariableRegistry("SphereController");

   private final RobotTools.SCSRobotFromInverseDynamicsRobotModel scsRobot;
   private final SphereControlToolbox controlToolbox;
   private final ExternalForcePoint externalForcePoint;

   private final BasicHeightController heightController;
   private final BasicPlanarController planarController;

   private final ICPProportionalController icpProportionalController;

   private final SimpleDCMPlan dcmPlan = new SimpleDCMPlan();

   public BasicSphereController(RobotTools.SCSRobotFromInverseDynamicsRobotModel scsRobot, SphereControlToolbox controlToolbox, ExternalForcePoint externalForcePoint)
   {
      this.scsRobot = scsRobot;
      this.controlToolbox = controlToolbox;
      this.externalForcePoint = externalForcePoint;

      ICPControlGains gains = new ICPControlGains();
      gains.setKpOrthogonalToMotion(3.0);
      gains.setKpParallelToMotion(2.0);

      icpProportionalController = new ICPProportionalController(gains, controlToolbox.getControlDT(), registry);

      heightController = new BasicHeightController(controlToolbox, registry);
      planarController = new BasicPlanarController(controlToolbox, registry);

   }

   private final Point2D planarForces = new Point2D();
   private final Vector3D forces = new Vector3D();


   @Override
   public void doControl()
   {
      scsRobot.updateJointPositions_SCS_to_ID();
      scsRobot.updateJointVelocities_SCS_to_ID();

      controlToolbox.update();

      dcmPlan.compute(controlToolbox.getYoTime().getDoubleValue());

      FramePoint2DReadOnly desiredCMP = icpProportionalController.doProportionalControl(null, controlToolbox.getCapturePoint2d(), dcmPlan.getDesiredDCMPosition(), null,
                                                                           dcmPlan.getDesiredDCMVelocity(), dcmPlan.getDesiredVRPPosition(), controlToolbox.getOmega0())
      heightController.doControl();
      planarController.doControl();
      planarController.getPlanarForces(planarForces);

      forces.set(planarForces.getX(), planarForces.getY(), heightController.getVerticalForce());

      externalForcePoint.setForce(forces);

      scsRobot.updateJointPositions_ID_to_SCS();
      scsRobot.updateJointVelocities_ID_to_SCS();
      scsRobot.updateJointTorques_ID_to_SCS();
   }

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
   public String getDescription()
   {
      return registry.getName();
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

}
