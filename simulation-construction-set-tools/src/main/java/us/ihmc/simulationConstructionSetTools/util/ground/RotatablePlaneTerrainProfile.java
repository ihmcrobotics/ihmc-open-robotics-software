package us.ihmc.simulationConstructionSetTools.util.ground;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex3DSupplier;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.geometry.shapes.FramePlane3d;
import us.ihmc.robotics.math.filters.AlphaFilteredWrappingYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.variable.YoVariable;

public class RotatablePlaneTerrainProfile implements GroundProfile3D, RobotController
{
   private static final ReferenceFrame WORLD_FRAME = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry("rotatablePlaneTerrainProfile");
   private final FramePose3D planePose = new FramePose3D(WORLD_FRAME);
   private final PoseReferenceFrame planeFrame = new PoseReferenceFrame("planeFrame", planePose);
   private final FramePlane3d plane = new FramePlane3d(planeFrame);
   private final FramePlane3d previousPlane = new FramePlane3d(WORLD_FRAME);
   private final YoGraphicPolygon floorGraphic;
   
   private final YoDouble ground_kp = new YoDouble("ground_kp", registry);
   private final YoDouble ground_kd =  new YoDouble("ground_kd", registry);
   private final YoDouble filteredDesiredGroundYawAlpha = new YoDouble("filteredDesiredGroundYawAlpha", registry);
   private final YoDouble filteredDesiredGroundPitchAlpha = new YoDouble("filteredDesiredGroundOrientationAlpha", registry);
   private final YoDouble filteredDesiredGroundRollAlpha = new YoDouble("filteredDesiredGroundRollAlpha", registry);
   
   private final YoFrameYawPitchRoll desiredGroundOrientation = new YoFrameYawPitchRoll("desiredGroundOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final AlphaFilteredWrappingYoVariable filteredDesiredGroundYaw = new AlphaFilteredWrappingYoVariable("filteredDesiredGroundYaw", "", registry, desiredGroundOrientation.getYoYaw(), filteredDesiredGroundYawAlpha, -Math.PI, Math.PI);
   private final AlphaFilteredWrappingYoVariable filteredDesiredGroundPitch = new AlphaFilteredWrappingYoVariable("filteredDesiredGroundPitch", "", registry, desiredGroundOrientation.getYoPitch(), filteredDesiredGroundPitchAlpha, -Math.PI, Math.PI);
   private final AlphaFilteredWrappingYoVariable filteredDesiredGroundRoll = new AlphaFilteredWrappingYoVariable("filteredDesiredGroundRoll", "", registry, desiredGroundOrientation.getYoRoll(), filteredDesiredGroundRollAlpha, -Math.PI, Math.PI);
   private final YoFrameYawPitchRoll filteredDesiredGroundOrientation = new YoFrameYawPitchRoll(filteredDesiredGroundYaw, filteredDesiredGroundPitch, filteredDesiredGroundRoll, ReferenceFrame.getWorldFrame());

   private YoFramePoint3D desiredGroundPosition = new YoFramePoint3D("desiredGroundPosition", WORLD_FRAME, registry);
   private final YoFramePoseUsingYawPitchRoll yoPlanePose = new YoFramePoseUsingYawPitchRoll(desiredGroundPosition, filteredDesiredGroundOrientation);
   private ArrayList<GroundContactPoint> groundContactPoints;
   
   public RotatablePlaneTerrainProfile(Point3D center, Robot robot, YoGraphicsListRegistry graphicsRegistry, double dt)
   {
      
      ground_kp.set(64000.0); 
      ground_kd.set(500.0);
      
      this.groundContactPoints = robot.getGroundContactPoints(0);
      
      filteredDesiredGroundYawAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(0.2, dt));
      filteredDesiredGroundPitchAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(0.2, dt));
      filteredDesiredGroundRollAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(0.2, dt));
      
      yoPlanePose.attachVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            yoPlanePose.getFramePose(planePose);
            planeFrame.setPoseAndUpdate(planePose);
         }
      });

      YoFrameConvexPolygon2D yoFrameConvexPolygon2d = new YoFrameConvexPolygon2D("floorGraphicPolygon", "", planeFrame, 4, registry);
      
      FramePoint3D p0 = new FramePoint3D(WORLD_FRAME, 1.0, 1.0, 0.0);
      FramePoint3D p1 = new FramePoint3D(WORLD_FRAME, -1.0, 1.0, 0.0);
      FramePoint3D p2 = new FramePoint3D(WORLD_FRAME, -1.0, -1.0, 0.0);
      FramePoint3D p3 = new FramePoint3D(WORLD_FRAME, 1.0, -1.0, 0.0);
      FramePoint3D[] framePoints = new FramePoint3D[]{p0, p1, p2, p3};
      
      yoFrameConvexPolygon2d.set(FrameVertex3DSupplier.asFrameVertex3DSupplier(framePoints));
      
      floorGraphic = new YoGraphicPolygon("floorGraphic", yoFrameConvexPolygon2d, yoPlanePose, 3.0, YoAppearance.DimGrey());
      graphicsRegistry.registerYoGraphic("ground", floorGraphic);
      
      updatePreviousPlane();
      robot.setController(this);
   }
   
   @Override
   public BoundingBox3D getBoundingBox()
   {
      return null;
   }

   FramePoint3D testPoint = new FramePoint3D(WORLD_FRAME);
   @Override
   public boolean isClose(double x, double y, double z)
   {
      testPoint.setIncludingFrame(WORLD_FRAME, x, y, z);
      testPoint.changeFrame(plane.getReferenceFrame());
      return plane.isOnOrBelow(testPoint);
   }

   FrameVector3D normalVector = new FrameVector3D(WORLD_FRAME);
   FramePoint2D xyPoint = new FramePoint2D(WORLD_FRAME);
   /**
    * Returns true if inside the ground object. If inside, must pack the intersection and normal. If not inside, packing those is optional.
    */
   @Override
   public boolean checkIfInside(double x, double y, double z, Point3D intersectionToPack, Vector3D normalToPack)
   {
      testPoint.setIncludingFrame(WORLD_FRAME, x, y, z);
      testPoint.changeFrame(plane.getReferenceFrame());
      boolean onOrBelow = plane.isOnOrBelow(testPoint);
      
      if(onOrBelow)
      {
         normalVector.setToNaN(plane.getReferenceFrame());
         plane.getNormal(normalVector);
         normalVector.changeFrame(WORLD_FRAME);
         normalToPack.set(normalVector);
         
         xyPoint.setToNaN(planeFrame);
         xyPoint.setIncludingFrame(testPoint);
         double zHeight = plane.getZOnPlane(xyPoint);
         
         testPoint.changeFrame(WORLD_FRAME);
         testPoint.setZ(zHeight);
         intersectionToPack.set(testPoint);
      }
      
      return onOrBelow;
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return null;
   }

   FrameVector3D v1 = new FrameVector3D(planeFrame);
   FrameVector3D v2 = new FrameVector3D(WORLD_FRAME);
   Vector3D v3 = new Vector3D();
   FramePoint3D p1 = new FramePoint3D(WORLD_FRAME);
   FramePoint3D p2 = new FramePoint3D(WORLD_FRAME);
   
   public void velocityAt(double x, double y, double z, Vector3D normal)
   {
      xyPoint.setIncludingFrame(WORLD_FRAME, x, y);
      double prevZ = previousPlane.getZOnPlane(xyPoint);
      
      p1.setIncludingFrame(WORLD_FRAME, x, y, z);
      p1.changeFrame(planeFrame);
      xyPoint.setToNaN(planeFrame);
      xyPoint.setIncludingFrame(p1);
      double currentZ = plane.getZOnPlane(xyPoint);
      
      v1.changeFrame(planeFrame);
      plane.getNormal(v1);
      v1.changeFrame(WORLD_FRAME);
      previousPlane.getNormal(v2);
      
      v3.add(v1, v2);
      v3.scale(-0.5);
//      
      v3.scale(currentZ - prevZ);
      normal.set(v3);
//      normal.set(0.0,0.0,v3.z);
   }
   
   public boolean hasMoved(Point3D position)
   {
      xyPoint.setIncludingFrame(WORLD_FRAME, position.getX(), position.getY());
      double prevZ = previousPlane.getZOnPlane(xyPoint);
      
      p1.setIncludingFrame(WORLD_FRAME, position);
      p1.changeFrame(planeFrame);
      xyPoint.setToNaN(planeFrame);
      xyPoint.setIncludingFrame(p1);
      double currentZ = plane.getZOnPlane(xyPoint);
      
      return currentZ - prevZ != 0;
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
   public String getName()
   {
      return null;
   }

   @Override
   public String getDescription()
   {
      return null;
   }

   private final FramePoint3D pointOnPlane = new FramePoint3D(WORLD_FRAME);
   private final Vector3D gcForce = new Vector3D();
   private final Vector3D gcVelocity = new Vector3D();
   @Override
   public void doControl()
   {
      updatePreviousPlane();
      
      filteredDesiredGroundYaw.update();
      filteredDesiredGroundPitch.update();
      filteredDesiredGroundRoll.update();
      
      for(int i = 0; i < groundContactPoints.size(); i++)
      {
         testPoint.setToNaN(WORLD_FRAME);
         GroundContactPoint groundContactPoint = groundContactPoints.get(i);
         testPoint.set(groundContactPoint.getYoPosition());

         testPoint.changeFrame(planeFrame);
         if(plane.isOnOrBelow(testPoint))
         {
            xyPoint.setIncludingFrame(planeFrame, testPoint.getX(),testPoint.getY());
            double zOnPlane = plane.getZOnPlane(xyPoint);
            pointOnPlane.setIncludingFrame(planeFrame, testPoint.getX(), testPoint.getY(), zOnPlane);
            pointOnPlane.changeFrame(WORLD_FRAME);
            
            groundContactPoint.getVelocity(gcVelocity);
            groundContactPoint.getForce(gcForce);
            
            testPoint.changeFrame(WORLD_FRAME);
            double zForce = ground_kp.getDoubleValue() * (pointOnPlane.getZ() - testPoint.getZ()) + ground_kd.getDoubleValue() *  gcVelocity.getZ();
            gcForce.setZ(gcForce.getZ() + zForce);
            groundContactPoint.setForce(gcForce);
         }
//         if (ef.z.val > water_z) ef.fz.val = 0.0;
//         else ef.fz.val = k_bouyancy.val * (water_z - ef.z.val) - b_bouyancy.val * ef.dz.val;
      }
   }

   private void updatePreviousPlane()
   {
      normalVector.changeFrame(plane.getReferenceFrame());
      plane.getNormal(normalVector);
      normalVector.changeFrame(WORLD_FRAME);
      previousPlane.setNormal(normalVector);
      
      testPoint.changeFrame(plane.getReferenceFrame());
      plane.getPoint(testPoint);
      testPoint.changeFrame(WORLD_FRAME);
      previousPlane.setPoint(testPoint);
   }
}