package us.ihmc.simulationconstructionset.util.ground;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.HeightMapWithNormals;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.shapes.FramePlane3d;
import us.ihmc.robotics.math.filters.AlphaFilteredWrappingYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Robot;

public class RotatablePlaneTerrainProfile implements GroundProfile3D, RobotController
{
   private static final ReferenceFrame WORLD_FRAME = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry("rotatablePlaneTerrainProfile");
   private final FramePose planePose = new FramePose(WORLD_FRAME);
   private final PoseReferenceFrame planeFrame = new PoseReferenceFrame("planeFrame", planePose);
   private final FramePlane3d plane = new FramePlane3d(planeFrame);
   private final FramePlane3d previousPlane = new FramePlane3d(WORLD_FRAME);
   private final YoGraphicPolygon floorGraphic;
   
   private final DoubleYoVariable ground_kp = new DoubleYoVariable("ground_kp", registry);
   private final DoubleYoVariable ground_kd =  new DoubleYoVariable("ground_kd", registry);
   private final DoubleYoVariable filteredDesiredGroundYawAlpha = new DoubleYoVariable("filteredDesiredGroundYawAlpha", registry);
   private final DoubleYoVariable filteredDesiredGroundPitchAlpha = new DoubleYoVariable("filteredDesiredGroundOrientationAlpha", registry);
   private final DoubleYoVariable filteredDesiredGroundRollAlpha = new DoubleYoVariable("filteredDesiredGroundRollAlpha", registry);
   
   private final YoFrameOrientation desiredGroundOrientation = new YoFrameOrientation("desiredGroundOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final AlphaFilteredWrappingYoVariable filteredDesiredGroundYaw = new AlphaFilteredWrappingYoVariable("filteredDesiredGroundYaw", "", registry, desiredGroundOrientation.getYaw(), filteredDesiredGroundYawAlpha, -Math.PI, Math.PI);
   private final AlphaFilteredWrappingYoVariable filteredDesiredGroundPitch = new AlphaFilteredWrappingYoVariable("filteredDesiredGroundPitch", "", registry, desiredGroundOrientation.getPitch(), filteredDesiredGroundPitchAlpha, -Math.PI, Math.PI);
   private final AlphaFilteredWrappingYoVariable filteredDesiredGroundRoll = new AlphaFilteredWrappingYoVariable("filteredDesiredGroundRoll", "", registry, desiredGroundOrientation.getRoll(), filteredDesiredGroundRollAlpha, -Math.PI, Math.PI);
   private final YoFrameOrientation filteredDesiredGroundOrientation = new YoFrameOrientation(filteredDesiredGroundYaw, filteredDesiredGroundPitch, filteredDesiredGroundRoll, ReferenceFrame.getWorldFrame());

   private YoFramePoint desiredGroundPosition = new YoFramePoint("desiredGroundPosition", WORLD_FRAME, registry);
   private final YoFramePose yoPlanePose = new YoFramePose(desiredGroundPosition, filteredDesiredGroundOrientation);
   private ArrayList<GroundContactPoint> groundContactPoints;
   
   public RotatablePlaneTerrainProfile(Point3d center, Robot robot, YoGraphicsListRegistry graphicsRegistry, double dt)
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
         public void variableChanged(YoVariable<?> v)
         {
            yoPlanePose.getFramePose(planePose);
            planeFrame.setPoseAndUpdate(planePose);
         }
      });

      YoFrameConvexPolygon2d yoFrameConvexPolygon2d = new YoFrameConvexPolygon2d("floorGraphicPolygon", "", planeFrame, 4, registry);
      
      FramePoint p0 = new FramePoint(WORLD_FRAME, 1.0, 1.0, 0.0);
      FramePoint p1 = new FramePoint(WORLD_FRAME, -1.0, 1.0, 0.0);
      FramePoint p2 = new FramePoint(WORLD_FRAME, -1.0, -1.0, 0.0);
      FramePoint p3 = new FramePoint(WORLD_FRAME, 1.0, -1.0, 0.0);
      FramePoint[] framePoints = new FramePoint[]{p0, p1, p2, p3};
      
      yoFrameConvexPolygon2d.setConvexPolygon2d(framePoints);
      
      floorGraphic = new YoGraphicPolygon("floorGraphic", yoFrameConvexPolygon2d, yoPlanePose, 3.0, YoAppearance.DimGrey());
      graphicsRegistry.registerYoGraphic("ground", floorGraphic);
      
      updatePreviousPlane();
      robot.setController(this);
   }
   
   @Override
   public BoundingBox3d getBoundingBox()
   {
      return null;
   }

   FramePoint testPoint = new FramePoint(WORLD_FRAME);
   @Override
   public boolean isClose(double x, double y, double z)
   {
      testPoint.setIncludingFrame(WORLD_FRAME, x, y, z);
      testPoint.changeFrame(plane.getReferenceFrame());
      return plane.isOnOrBelow(testPoint);
   }

   FrameVector normalVector = new FrameVector(WORLD_FRAME);
   FramePoint2d xyPoint = new FramePoint2d(WORLD_FRAME);
   /**
    * Returns true if inside the ground object. If inside, must pack the intersection and normal. If not inside, packing those is optional.
    */
   @Override
   public boolean checkIfInside(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack)
   {
      testPoint.setIncludingFrame(WORLD_FRAME, x, y, z);
      testPoint.changeFrame(plane.getReferenceFrame());
      boolean onOrBelow = plane.isOnOrBelow(testPoint);
      
      if(onOrBelow)
      {
         normalVector.setToNaN(plane.getReferenceFrame());
         plane.getNormal(normalVector);
         normalVector.changeFrame(WORLD_FRAME);
         normalVector.getVector(normalToPack);
         
         xyPoint.setToNaN(planeFrame);
         testPoint.getFramePoint2d(xyPoint);
         double zHeight = plane.getZOnPlane(xyPoint);
         
         testPoint.changeFrame(WORLD_FRAME);
         testPoint.setZ(zHeight);
         testPoint.get(intersectionToPack);
      }
      
      return onOrBelow;
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return null;
   }

   FrameVector v1 = new FrameVector(planeFrame);
   FrameVector v2 = new FrameVector(WORLD_FRAME);
   Vector3d v3 = new Vector3d();
   FramePoint p1 = new FramePoint(WORLD_FRAME);
   FramePoint p2 = new FramePoint(WORLD_FRAME);
   
   public void velocityAt(double x, double y, double z, Vector3d normal)
   {
      xyPoint.setIncludingFrame(WORLD_FRAME, x, y);
      double prevZ = previousPlane.getZOnPlane(xyPoint);
      
      p1.setIncludingFrame(WORLD_FRAME, x, y, z);
      p1.changeFrame(planeFrame);
      xyPoint.setToNaN(planeFrame);
      p1.getFramePoint2d(xyPoint);
      double currentZ = plane.getZOnPlane(xyPoint);
      
      v1.changeFrame(planeFrame);
      plane.getNormal(v1);
      v1.changeFrame(WORLD_FRAME);
      previousPlane.getNormal(v2);
      
      v3.add(v1.getVector(), v2.getVector());
      v3.scale(-0.5);
//      
      v3.scale(currentZ - prevZ);
      normal.set(v3);
//      normal.set(0.0,0.0,v3.z);
   }
   
   public boolean hasMoved(Point3d position)
   {
      xyPoint.setIncludingFrame(WORLD_FRAME, position.getX(), position.getY());
      double prevZ = previousPlane.getZOnPlane(xyPoint);
      
      p1.setIncludingFrame(WORLD_FRAME, position);
      p1.changeFrame(planeFrame);
      xyPoint.setToNaN(planeFrame);
      p1.getFramePoint2d(xyPoint);
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

   private final FramePoint pointOnPlane = new FramePoint(WORLD_FRAME);
   private final Vector3d gcForce = new Vector3d();
   private final Vector3d gcVelocity = new Vector3d();
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
         groundContactPoint.getYoPosition().getFrameTuple(testPoint);

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
      previousPlane.setNormal(normalVector.getVector());
      
      testPoint.changeFrame(plane.getReferenceFrame());
      plane.getPoint(testPoint);
      testPoint.changeFrame(WORLD_FRAME);
      previousPlane.setPoint(testPoint.getPoint());
   }
}