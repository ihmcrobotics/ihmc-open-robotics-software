package us.ihmc.exampleSimulations.harmonograph;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FunctionToIntegrate;
import us.ihmc.simulationconstructionset.GimbalJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.UniversalJoint;

public class HarmonographRobot extends Robot
{
   private static final long serialVersionUID = 3039595027889423110L;
   
   public static final double INCHES = 0.0254;
   public static final double FEET = 12.0 * INCHES;
   
   private static final double XY_PENDULUM_OFFSET = 23.25 * INCHES;

   private static final double PEN_HEIGHT = 4.0 * INCHES;
   private static final double PEN_RADIUS = 0.25 * INCHES;
   
   private static final double TABLE_LENGTH = 4.0 * FEET;
   private static final double TABLE_WIDTH = 4.0 * FEET;
   private static final double TABLE_THICKNESS = 0.75 * INCHES;
   
   private static final double TABLE_CENTER_X_OFFSET = XY_PENDULUM_OFFSET/2.0;
   private static final double TABLE_CENTER_Y_OFFSET = XY_PENDULUM_OFFSET/2.0;

   private static final double TABLE_HEIGHT = (36.0 + 0.75 + 1.5) * INCHES;
   
   private static final double DESK_PENDULUM_LENGTH = 48.0 * INCHES; //39.0 * INCHES;
   private static final double DESK_PENDULUM_RADIUS = 1.0 * INCHES;
   private static final double DESK_PENDULUM_MASS = 25.0;
   
   private static final double DESK_WEIGHTS_HEIGHT = 3.0 * INCHES;
   private static final double DESK_WEIGHTS_RADIUS = 4.0 * INCHES;
   
   private static final double DESK_PENDULUM_RADIUS_GRYRATION_X = DESK_WEIGHTS_RADIUS * 0.3;
   private static final double DESK_PENDULUM_RADIUS_GRYRATION_Y = DESK_WEIGHTS_RADIUS * 0.3;
   private static final double DESK_PENDULUM_RADIUS_GRYRATION_Z = DESK_WEIGHTS_HEIGHT * 0.3;
   
   private static final double DESK_PENDULUM_PERCENT_DOWN = 0.8;
   private static final double DESK_PENDULUM_OFFSET_Z = -DESK_PENDULUM_PERCENT_DOWN * DESK_PENDULUM_LENGTH;
   private static final double DESK_WEIGHTS_DISTANCE_FROM_BOTTOM = 0.05;

   private static final double DESK_HEIGHT = (1.0 - DESK_PENDULUM_PERCENT_DOWN) * DESK_PENDULUM_LENGTH;
   private static final double DESK_PENDULUM_COM_Z = -DESK_PENDULUM_PERCENT_DOWN * DESK_PENDULUM_LENGTH + DESK_WEIGHTS_DISTANCE_FROM_BOTTOM;

   private static final double DESK_WIDTH = 14.0 * INCHES;
   private static final double DESK_LENGTH = 14.0 * INCHES;
   private static final double DESK_THICKNESS = 0.25 * INCHES;

   private static final double XY_PENDULUM_MASS = DESK_PENDULUM_MASS;
   private static final double XY_PENDULUM_RADIUS_GRYRATION_X = DESK_PENDULUM_RADIUS_GRYRATION_X;
   private static final double XY_PENDULUM_RADIUS_GRYRATION_Y = DESK_PENDULUM_RADIUS_GRYRATION_Y;
   private static final double XY_PENDULUM_RADIUS_GRYRATION_Z = DESK_PENDULUM_RADIUS_GRYRATION_Z;

   private static final double XY_PENDULUM_LENGTH = DESK_PENDULUM_LENGTH; // * 9.0/16.0; //1.0; //3.0 / 4.0; //4.0 / 3.0;
   private static final double XY_WEIGHTS_DISTANCE_FROM_BOTTOM = DESK_WEIGHTS_DISTANCE_FROM_BOTTOM;
   private static final double XY_PENDULUM_PERCENT_DOWN = 0.8;
   private static final double XY_PENDULUM_COM_Z = -XY_PENDULUM_PERCENT_DOWN * XY_PENDULUM_LENGTH + XY_WEIGHTS_DISTANCE_FROM_BOTTOM;
   private static final double XY_PENDULUM_OFFSET_Z = -XY_PENDULUM_PERCENT_DOWN * XY_PENDULUM_LENGTH;
   private static final double XY_PENDULUM_PIVOT_TO_TOP = XY_PENDULUM_LENGTH + XY_PENDULUM_OFFSET_Z + PEN_HEIGHT * 2.0;

   private static final double XY_PENDULUM_RADIUS = DESK_PENDULUM_RADIUS;
   private static final double XY_WEIGHTS_HEIGHT = DESK_WEIGHTS_HEIGHT;
   private static final double XY_WEIGHTS_RADIUS = DESK_WEIGHTS_RADIUS;

   private static final double ARM_LENGTH = XY_PENDULUM_OFFSET;
   private static final double ARM_RADIUS = 0.01;

   private static final double DEGREES = Math.PI / 180.0;
   private static final double MAXIMUM_ANGLE = 11.0 * DEGREES;
   private static final double K_ANGLE_LIMIT = 1000.0;
   private static final double B_ANGLE_LIMIT = 100.0;

   
   private final DoubleYoVariable kpArmWeld, kdArmWeld, kpPenOnDesk;
   private final BooleanYoVariable penIsAboveDesk;
   private final DoubleYoVariable penToDeskDistance;
   private final BooleanYoVariable clearPoints;

   private final DoubleYoVariable rotationalKineticEnergy, translationalKineticEnergy, gravitationalPotentialEnergy, totalEnergy;
   
   private final PinJoint deskPendulumYJoint;
   private final HarmonographPaperJPanel harmonographPaperJPanel;
   
   public HarmonographRobot(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super("Harmonograph");
      
      harmonographPaperJPanel = new HarmonographPaperJPanel();
      YoVariableRegistry registry = this.getRobotsYoVariableRegistry();
      
      kpArmWeld = new DoubleYoVariable("kpArmWeld", registry);
      kdArmWeld = new DoubleYoVariable("kdArmWeld", registry);
      kpPenOnDesk = new DoubleYoVariable("kpPenOnDesk", registry);
      
      penIsAboveDesk = new BooleanYoVariable("penIsAboveDesk", registry);
      penToDeskDistance = new DoubleYoVariable("penToDeskDistance", registry);
      
      clearPoints = new BooleanYoVariable("clearPoints", registry);

      rotationalKineticEnergy = new DoubleYoVariable("rotationalKineticEnergy", registry);
      translationalKineticEnergy = new DoubleYoVariable("translationalKineticEnergy", registry);
      gravitationalPotentialEnergy = new DoubleYoVariable("gravitationalPotentialEnergy", registry);
      
      totalEnergy = new DoubleYoVariable("totalEnergy", registry);
      
      kpArmWeld.set(1000.0);
      kdArmWeld.set(100.0);
      kpPenOnDesk.set(1000.0);
      
      // Desk
      Graphics3DObject tableLinkGraphics = new Graphics3DObject();
      tableLinkGraphics.translate(TABLE_CENTER_X_OFFSET, TABLE_CENTER_Y_OFFSET, TABLE_HEIGHT - TABLE_THICKNESS/2.0);
      tableLinkGraphics.addCube(TABLE_LENGTH, TABLE_WIDTH, TABLE_THICKNESS, YoAppearance.Brown());
      this.addStaticLinkGraphics(tableLinkGraphics);
      
      // Desk Pendulum
      PinJoint deskPendulumXJoint = new PinJoint("deskPendulumX", new Vector3D(0.0, 0.0, TABLE_HEIGHT), this, Axis.X);
      Link universalLink = createDeskPendulumUniversalLink();
      deskPendulumXJoint.setLink(universalLink);
      deskPendulumXJoint.setLimitStops(-MAXIMUM_ANGLE, MAXIMUM_ANGLE, K_ANGLE_LIMIT, B_ANGLE_LIMIT);
      
      this.addRootJoint(deskPendulumXJoint);
      
      deskPendulumYJoint = new PinJoint("deskPendulumY", new Vector3D(), this, Axis.Y);
      Link deskPendulumLink = createDeskPendulumLink();
      deskPendulumYJoint.setLink(deskPendulumLink);
      deskPendulumYJoint.setLimitStops(-MAXIMUM_ANGLE, MAXIMUM_ANGLE, K_ANGLE_LIMIT, B_ANGLE_LIMIT);

      final ExternalForcePoint deskCenterExternalForcePoint = new ExternalForcePoint("ef_deskCenter", new Vector3D(0.0, 0.0, DESK_HEIGHT), this);

//      KinematicPoint deskCornerOne = new KinematicPoint("kp_deskOne", new Vector3d(DESK_LENGTH/2.0, DESK_WIDTH/2.0, DESK_HEIGHT), this);
//      KinematicPoint deskCornerTwo = new KinematicPoint("kp_deskTwo", new Vector3d(-DESK_LENGTH/2.0, DESK_WIDTH/2.0, DESK_HEIGHT), this);
//      KinematicPoint deskCornerThree = new KinematicPoint("kp_deskThree", new Vector3d(-DESK_LENGTH/2.0, -DESK_WIDTH/2.0, DESK_HEIGHT), this);
//      KinematicPoint deskCornerFour = new KinematicPoint("kp_deskFour", new Vector3d(DESK_LENGTH/2.0, -DESK_WIDTH/2.0, DESK_HEIGHT), this);
      
      deskPendulumYJoint.addKinematicPoint(deskCenterExternalForcePoint);
//      deskPendulumYJoint.addKinematicPoint(deskCornerOne);
//      deskPendulumYJoint.addKinematicPoint(deskCornerTwo);
//      deskPendulumYJoint.addKinematicPoint(deskCornerThree);
//      deskPendulumYJoint.addKinematicPoint(deskCornerFour);
      
      deskPendulumXJoint.addJoint(deskPendulumYJoint);
      
      // xPendulum
      PinJoint xPendulumJoint = new PinJoint("xPendulum", new Vector3D(XY_PENDULUM_OFFSET, 0.0, TABLE_HEIGHT), this, Axis.Y);
      Link xPendulumLink = createXYPendulumLink();
      xPendulumJoint.setLink(xPendulumLink);
      xPendulumJoint.setLimitStops(-MAXIMUM_ANGLE, MAXIMUM_ANGLE, K_ANGLE_LIMIT, B_ANGLE_LIMIT);
      this.addRootJoint(xPendulumJoint);
      
//      GimbalJoint xPendulumArmJoint = new GimbalJoint("xGim1", "xGim2", "xGim3", new Vector3d(0.0, 0.0, XY_PENDULUM_PIVOT_TO_TOP), this, Axis.X, Axis.Y, Axis.Z);
      UniversalJoint xPendulumArmJoint = new UniversalJoint("xUni1", "xUni2", new Vector3D(0.0, 0.0, XY_PENDULUM_PIVOT_TO_TOP), this, Axis.Y, Axis.Z);
      Link xPendulumArmLink = createXPendulumArmLink();
      xPendulumArmJoint.setLink(xPendulumArmLink);
      xPendulumJoint.addJoint(xPendulumArmJoint);
      
      xPendulumArmJoint.setDamping(0.1); //1.0);
      
      final ExternalForcePoint xArmExternalForcePoint = new ExternalForcePoint("ef_xArm", new Vector3D(-ARM_LENGTH, 0.0, 0.0), this);
      xPendulumArmJoint.addExternalForcePoint(xArmExternalForcePoint);
      
      final ExternalForcePoint penExternalForcePoint = new ExternalForcePoint("ef_pen", new Vector3D(-ARM_LENGTH, 0.0, -PEN_HEIGHT), this);
      xPendulumArmJoint.addExternalForcePoint(penExternalForcePoint);
      
   // yPendulum
      PinJoint yPendulumJoint = new PinJoint("yPendulum", new Vector3D(0.0, XY_PENDULUM_OFFSET, TABLE_HEIGHT), this, Axis.X);
      Link yPendulumLink = createXYPendulumLink();
      yPendulumJoint.setLink(yPendulumLink);
      yPendulumJoint.setLimitStops(-MAXIMUM_ANGLE, MAXIMUM_ANGLE, K_ANGLE_LIMIT, B_ANGLE_LIMIT);
      this.addRootJoint(yPendulumJoint);
      
      GimbalJoint yPendulumArmGimbal = new GimbalJoint("yGim1", "yGim2", "yGim3", new Vector3D(0.0, 0.0, XY_PENDULUM_PIVOT_TO_TOP), this, Axis.X, Axis.Y, Axis.Z);
      Link yPendulumArmLink = createYPendulumArmLink();
      yPendulumArmGimbal.setLink(yPendulumArmLink);
      yPendulumJoint.addJoint(yPendulumArmGimbal);
      
      final ExternalForcePoint yArmExternalForcePoint = new ExternalForcePoint("ef_yArm", new Vector3D(0.0, -ARM_LENGTH, 0.0), this);
      yPendulumArmGimbal.addExternalForcePoint(yArmExternalForcePoint);
      
      
      // Initial conditions:
      deskPendulumXJoint.setInitialState(0.5, 0.07);
      deskPendulumYJoint.setInitialState(0.0, 0.3);
      xPendulumJoint.setInitialState(0.1, 0.6);
      yPendulumJoint.setInitialState(0.4, 0.0);
      
      
      // Visualizers:
      final YoGraphicPosition penPositionViz = new YoGraphicPosition("penPosition", "", registry, 0.01, YoAppearance.Purple());
      final YoGraphicPosition deskCenterViz = new YoGraphicPosition("deskCenter", "", registry, 0.01, YoAppearance.Orange());
      final YoGraphicPosition penProjectionOntoDeskViz = new YoGraphicPosition("penProjectionOntoDesk", "", registry, 0.01, YoAppearance.DarkBlue());
//      final BagOfBalls bagOfBalls = new BagOfBalls(registry, yoGraphicsListRegistry);
      
      YoGraphicsList yoGraphicsList = new YoGraphicsList("PenAndDesk");
      yoGraphicsList.add(penPositionViz);
      yoGraphicsList.add(deskCenterViz);
      yoGraphicsList.add(penProjectionOntoDeskViz);
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      
      rotationalKineticEnergy.set(this.computeRotationalKineticEnergy());
      translationalKineticEnergy.set(this.computeTranslationalKineticEnergy());
      
      
      FunctionToIntegrate weldAndPenFunctionToIntegrate = new FunctionToIntegrate()
      {
         private final Point3D xArmPosition = new Point3D();
         private final Point3D yArmPosition = new Point3D();
         private final Vector3D xArmVelocity = new Vector3D();
         private final Vector3D yArmVelocity = new Vector3D();
         private final RigidBodyTransform transformFromDeskToWorld = new RigidBodyTransform();
         private final RigidBodyTransform transformFromWorldToDesk = new RigidBodyTransform();
         
         private final Point3D penPosition = new Point3D();
         private final Point3D penProjectionInDeskFrame = new Point3D();
         private final Point3D deskCenterPosition = new Point3D();
         
         private final Vector3D deskSurfaceNormal = new Vector3D();
         private final Vector3D deskCenterToPen = new Vector3D();
         private final Vector3D penProjectionOntoDeskNormal = new Vector3D();
         private final Point3D penProjectionOntoDesk = new Point3D();
         
         public int getVectorSize()
         {
            return 0;
         }
         
         public DoubleYoVariable[] getOutputVariables()
         {
            return new DoubleYoVariable[]{};
         }
         
         public double[] computeDerivativeVector()
         {
            xArmExternalForcePoint.getPosition(xArmPosition);
            xArmExternalForcePoint.getVelocity(xArmVelocity);
            
            yArmExternalForcePoint.getPosition(yArmPosition);
            yArmExternalForcePoint.getVelocity(yArmVelocity);
            
            Vector3D xArmToYArm = new Vector3D();
            xArmToYArm.sub(yArmPosition, xArmPosition);
            
            Vector3D xArmToYArmVelocity = new Vector3D();
            xArmToYArmVelocity.sub(yArmVelocity, xArmVelocity);
            
            Vector3D armForcesKp = new Vector3D();
            Vector3D armForcesKd = new Vector3D();
            
            armForcesKp.set(xArmToYArm);
            armForcesKp.scale(kpArmWeld.getDoubleValue());
            
            armForcesKd.set(xArmToYArmVelocity);
            armForcesKd.scale(kdArmWeld.getDoubleValue());
            
            Vector3D armForces = new Vector3D();
            armForces.add(armForcesKp, armForcesKd);
            
            xArmExternalForcePoint.setForce(armForces);
            armForces.negate();
            yArmExternalForcePoint.setForce(armForces);
            
            
            // Pen onto desk:
            
            deskPendulumYJoint.getTransformToWorld(transformFromDeskToWorld);
            transformFromWorldToDesk.set(transformFromDeskToWorld);
            transformFromWorldToDesk.invert();
            
            deskSurfaceNormal.set(0.0, 0.0, 1.0);
            transformFromDeskToWorld.transform(deskSurfaceNormal);
            
            deskCenterExternalForcePoint.getPosition(deskCenterPosition);
            penExternalForcePoint.getPosition(penPosition);
            
            deskCenterToPen.set(penPosition);
            deskCenterToPen.sub(deskCenterPosition);
            
            penToDeskDistance.set(deskCenterToPen.dot(deskSurfaceNormal));
            penIsAboveDesk.set(penToDeskDistance.getDoubleValue() > 0.0);
            
            penProjectionOntoDeskNormal.set(deskSurfaceNormal);
            penProjectionOntoDeskNormal.scale(penToDeskDistance.getDoubleValue());
            
            penProjectionOntoDesk.set(penPosition);
            penProjectionOntoDesk.sub(penProjectionOntoDeskNormal);
            
            penProjectionInDeskFrame.set(penProjectionOntoDesk);
            transformFromWorldToDesk.transform(penProjectionInDeskFrame);
            
            // Only apply a force on the pen, not the desk. Doesn't follow Newton's third law, but the force is probably pretty small anyway.
            // Maybe later we'll track the other force point, and perhaps even check if energy is conserved...
            
            if (!penIsAboveDesk.getBooleanValue())
            {
               Vector3D forceOnPen = new Vector3D();
               forceOnPen.set(penProjectionOntoDeskNormal);
               forceOnPen.scale(-kpPenOnDesk.getDoubleValue());
               double maxPenForce = 5.0;
               
               if (forceOnPen.length() > maxPenForce)
               {
                  forceOnPen.normalize();
                  forceOnPen.scale(maxPenForce);
               }
               penExternalForcePoint.setForce(forceOnPen);
               
               if (getTime() > 10.0) harmonographPaperJPanel.addPoint(penProjectionInDeskFrame);
//               bagOfBalls.setBall(new FramePoint(ReferenceFrame.getWorldFrame(), penPosition));
            }
            else
            {
               penExternalForcePoint.setForce(new Vector3D());
            }
            
            if (clearPoints.getBooleanValue())
            {
               harmonographPaperJPanel.clearPoints();
               clearPoints.set(false);
            }
            
            // Visualizers:
            penPositionViz.setPosition(penPosition.getX(), penPosition.getY(), penPosition.getZ());
            deskCenterViz.setPosition(deskCenterPosition.getX(), deskCenterPosition.getY(), deskCenterPosition.getZ());
            penProjectionOntoDeskViz.setPosition(penProjectionOntoDesk.getX(), penProjectionOntoDesk.getY(), penProjectionOntoDesk.getZ());
            
            computeEnergies();
            
            return new double[]{};
         }
      };
      
      this.addFunctionToIntegrate(weldAndPenFunctionToIntegrate);

   }
   
   public HarmonographPaperJPanel getHarmonographPaperJPanel()
   {
      return harmonographPaperJPanel;
   }

   private Link createXPendulumArmLink()
   {
      Link xPendulumArmGimbalLink = new Link("xPendulumArmGimbalLink");
      xPendulumArmGimbalLink.setMassAndRadiiOfGyration(0.05, 0.02, 0.02, 0.02);
      xPendulumArmGimbalLink.setComOffset(-ARM_LENGTH/2.0, 0.0, 0.0);
      Graphics3DObject xPendulumArmGimbalLinkGraphics = new Graphics3DObject();
      xPendulumArmGimbalLinkGraphics.addSphere(0.04, YoAppearance.Red());
      xPendulumArmGimbalLinkGraphics.rotate(-Math.PI/2.0, Axis.Y);
      xPendulumArmGimbalLinkGraphics.addCylinder(ARM_LENGTH, ARM_RADIUS, YoAppearance.Pink());
      
      // Pen:
      xPendulumArmGimbalLinkGraphics.identity();
      xPendulumArmGimbalLinkGraphics.translate(-ARM_LENGTH, 0.0, -PEN_HEIGHT);
      xPendulumArmGimbalLinkGraphics.addCylinder(PEN_HEIGHT, PEN_RADIUS);

      
      xPendulumArmGimbalLink.setLinkGraphics(xPendulumArmGimbalLinkGraphics); 
      
      return xPendulumArmGimbalLink;
   }
   
   private Link createYPendulumArmLink()
   {
      Link yPendulumArmGimbalLink = new Link("yPendulumArmGimbalLink");
      yPendulumArmGimbalLink.setMassAndRadiiOfGyration(0.05, 0.02, 0.02, 0.02);
      yPendulumArmGimbalLink.setComOffset(0.0, -ARM_LENGTH/2.0, 0.0);
      Graphics3DObject yPendulumArmGimbalLinkGraphics = new Graphics3DObject();
      yPendulumArmGimbalLinkGraphics.addSphere(0.04, YoAppearance.Red());
      yPendulumArmGimbalLinkGraphics.rotate(Math.PI/2.0, Axis.X);
      yPendulumArmGimbalLinkGraphics.addCylinder(ARM_LENGTH, ARM_RADIUS, YoAppearance.Pink());
      yPendulumArmGimbalLink.setLinkGraphics(yPendulumArmGimbalLinkGraphics);
      return yPendulumArmGimbalLink;
   }

   private Link createDeskPendulumUniversalLink()
   {
      Link universalLink = new Link("universal");
      universalLink.setMassAndRadiiOfGyration(0.1, 0.1, 0.1, 0.1);
      Graphics3DObject universalLinkGraphics = new Graphics3DObject();
      universalLinkGraphics.addSphere(0.03);
      universalLink.setLinkGraphics(universalLinkGraphics);
      return universalLink;
   }

   private Link createDeskPendulumLink()
   {
      Link deskPendulumLink = new Link("deskPendulum");
      deskPendulumLink.setMassAndRadiiOfGyration(DESK_PENDULUM_MASS, DESK_PENDULUM_RADIUS_GRYRATION_X, DESK_PENDULUM_RADIUS_GRYRATION_Y, DESK_PENDULUM_RADIUS_GRYRATION_Z);
      deskPendulumLink.setComOffset(new Vector3D(0.0, 0.0, DESK_PENDULUM_COM_Z));
      
      Graphics3DObject deskPendulumLinkGraphics = new Graphics3DObject();
      deskPendulumLinkGraphics.translate(new Vector3D(0.0, 0.0, DESK_PENDULUM_OFFSET_Z));
      deskPendulumLinkGraphics.addCylinder(DESK_PENDULUM_LENGTH, DESK_PENDULUM_RADIUS);
      deskPendulumLinkGraphics.translate(0.0, 0.0, DESK_WEIGHTS_DISTANCE_FROM_BOTTOM);
      deskPendulumLinkGraphics.addCylinder(DESK_WEIGHTS_HEIGHT, DESK_WEIGHTS_RADIUS);
      deskPendulumLinkGraphics.identity();
      deskPendulumLinkGraphics.translate(0.0, 0.0, DESK_HEIGHT);
      deskPendulumLinkGraphics.addCube(DESK_WIDTH, DESK_LENGTH, DESK_THICKNESS);
      
      deskPendulumLink.setLinkGraphics(deskPendulumLinkGraphics);
      return deskPendulumLink;
   }

   private Link createXYPendulumLink()
   {
      Link xPendulumLink = new Link("xPendulum");
      xPendulumLink.setMassAndRadiiOfGyration(XY_PENDULUM_MASS, XY_PENDULUM_RADIUS_GRYRATION_X, XY_PENDULUM_RADIUS_GRYRATION_Y, XY_PENDULUM_RADIUS_GRYRATION_Z);
      xPendulumLink.setComOffset(new Vector3D(0.0, 0.0, XY_PENDULUM_COM_Z));
      
      Graphics3DObject xPendulumLinkGraphics = new Graphics3DObject();
     
      xPendulumLinkGraphics.translate(new Vector3D(0.0, 0.0, XY_PENDULUM_OFFSET_Z));
      xPendulumLinkGraphics.addCylinder(XY_PENDULUM_LENGTH, XY_PENDULUM_RADIUS);
      xPendulumLinkGraphics.translate(0.0, 0.0, XY_WEIGHTS_DISTANCE_FROM_BOTTOM);
      xPendulumLinkGraphics.addCylinder(XY_WEIGHTS_HEIGHT, XY_WEIGHTS_RADIUS);
      xPendulumLinkGraphics.translate(0.0, 0.0, XY_PENDULUM_LENGTH-XY_WEIGHTS_DISTANCE_FROM_BOTTOM);
      xPendulumLinkGraphics.addSphere(0.03);
      
      xPendulumLink.setLinkGraphics(xPendulumLinkGraphics);
      return xPendulumLink;
   }

   
   private final void computeEnergies()
   {
      rotationalKineticEnergy.set(this.computeRotationalKineticEnergy());
      translationalKineticEnergy.set(this.computeTranslationalKineticEnergy());
      gravitationalPotentialEnergy.set(this.computeGravitationalPotentialEnergy());
      
      totalEnergy.set(rotationalKineticEnergy.getDoubleValue() + translationalKineticEnergy.getDoubleValue() + gravitationalPotentialEnergy.getDoubleValue());
      
   }
   
   
   

}
