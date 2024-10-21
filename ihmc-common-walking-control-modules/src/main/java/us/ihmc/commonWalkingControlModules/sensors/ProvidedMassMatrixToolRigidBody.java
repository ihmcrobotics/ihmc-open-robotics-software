package us.ihmc.commonWalkingControlModules.sensors;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * @author unknownid
 *
 */
public class ProvidedMassMatrixToolRigidBody
{
   private final YoRegistry registry;

   private final RigidBodyBasics toolBody;

   private final ReferenceFrame handFixedFrame;
   private final ReferenceFrame handControlFrame;

   private final YoFramePoint3D objectCenterOfMass;
   private final YoFramePoint3D objectCenterOfMassInWorld;
   private final YoFrameVector3D objectForceInWorld;

   private final YoDouble objectMass;

   private final double gravity;
   private final SixDoFJoint toolJoint;
   private final ReferenceFrame elevatorFrame;

   private final FramePoint3D temporaryPoint = new FramePoint3D();
   private final FrameVector3D temporaryVector = new FrameVector3D();
   private final SpatialAcceleration toolAcceleration = new SpatialAcceleration();

   public ProvidedMassMatrixToolRigidBody(RobotSide robotSide, final FullHumanoidRobotModel fullRobotModel, double gravity, YoRegistry parentRegistry,
                                          YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      String name = robotSide.getCamelCaseNameForStartOfExpression() + "Tool";
      this.registry = new YoRegistry(name);
      this.gravity = gravity;

      this.handFixedFrame = fullRobotModel.getHand(robotSide).getBodyFixedFrame();
      this.handControlFrame = fullRobotModel.getHandControlFrame(robotSide);

      this.elevatorFrame = fullRobotModel.getElevatorFrame();

      this.toolJoint = new SixDoFJoint(name + "Joint", fullRobotModel.getElevator());
      this.toolBody = new RigidBody(name + "Body", toolJoint, new Matrix3D(), 0.0, new RigidBodyTransform());

      objectCenterOfMass = new YoFramePoint3D(name + "CoMOffset", handControlFrame, registry);
      objectMass = new YoDouble(name + "ObjectMass", registry);
      objectForceInWorld = new YoFrameVector3D(name + "Force", ReferenceFrame.getWorldFrame(), registry);

      this.objectCenterOfMassInWorld = new YoFramePoint3D(name + "CoMInWorld", ReferenceFrame.getWorldFrame(), registry);

      if (yoGraphicsListRegistry != null)
      {

         YoGraphicsList yoGraphicsList = new YoGraphicsList(name);
         YoGraphic comViz = new YoGraphicPosition(name + "CenterOfMassViz", objectCenterOfMassInWorld, 0.05, YoAppearance.Red());
         yoGraphicsList.add(comViz);

         YoGraphic vectorViz = new YoGraphicVector(name + "ForceViz", objectCenterOfMassInWorld, objectForceInWorld, YoAppearance.Yellow());
         yoGraphicsList.add(vectorViz);

         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      }
      parentRegistry.addChild(registry);
   }

   private boolean hasBeenInitialized = false;

   private void initialize()
   {
      hasBeenInitialized = true;
   }

   private final FramePoint3D toolFramePoint = new FramePoint3D();

   public void update()
   {
      toolBody.getInertia().setMass(objectMass.getDoubleValue());

      temporaryPoint.setIncludingFrame(objectCenterOfMass);
      temporaryPoint.changeFrame(toolBody.getBodyFixedFrame());
      toolBody.setCenterOfMass(temporaryPoint);

      // Visualization
      toolFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      objectCenterOfMassInWorld.set(toolFramePoint);
   }

   public void control(SpatialAccelerationReadOnly handSpatialAccelerationVector, Wrench toolWrench)
   {
      if (!hasBeenInitialized)
      {
         update();
         initialize();
      }

      update();

      temporaryVector.setIncludingFrame(elevatorFrame, 0.0, 0.0, gravity);
      temporaryVector.changeFrame(handSpatialAccelerationVector.getReferenceFrame());
      toolAcceleration.setIncludingFrame(handSpatialAccelerationVector);
      toolAcceleration.getLinearPart().add(temporaryVector);
      toolAcceleration.changeFrame(toolBody.getBodyFixedFrame());

      // TODO: Take relative acceleration between uTorsoCoM and elevator in account
      toolAcceleration.setBaseFrame(elevatorFrame);
      toolAcceleration.setBodyFrame(toolBody.getBodyFixedFrame());

      toolWrench.setToZero(handFixedFrame, handFixedFrame);
      toolBody.getInertia().computeDynamicWrench(toolAcceleration, toolBody.getBodyFixedFrame().getTwistOfFrame(), toolWrench);

      toolWrench.negate();
      toolWrench.changeFrame(handFixedFrame);
      toolWrench.setBodyFrame(handFixedFrame);

      // Visualization
      temporaryVector.setIncludingFrame(handFixedFrame, toolWrench.getLinearPartX(), toolWrench.getLinearPartY(), toolWrench.getLinearPartZ());
      temporaryVector.changeFrame(ReferenceFrame.getWorldFrame());
      temporaryVector.scale(0.01);
      objectForceInWorld.set(temporaryVector);

   }

   public void setMass(double mass)
   {
      objectMass.set(mass);
   }
}
