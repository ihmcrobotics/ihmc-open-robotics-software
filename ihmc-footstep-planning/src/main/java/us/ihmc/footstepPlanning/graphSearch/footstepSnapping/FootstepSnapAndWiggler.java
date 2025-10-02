package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.commonWalkingControlModules.polygonWiggling.StepConstraintPolygonTools;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.footstepPlanning.graphSearch.EnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapSnapWiggler;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

public class FootstepSnapAndWiggler implements FootstepSnapperReadOnly
{
   private final SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame;
   private final DefaultFootstepPlannerParametersReadOnly parameters;

   private final WiggleParameters wiggleParameters = new WiggleParameters();
   private double flatGroundHeight = 0.0;

   private final HashSet<DiscreteFootstep> snappedFootsteps = new HashSet<>();
   private final HashMap<DiscreteFootstep, FootstepSnapData> manuallySnappedFootsteps = new HashMap<>();

   private final EnvironmentHandler environmentHandler;

   private final HeightMapPolygonSnapper heightMapSnapper = new HeightMapPolygonSnapper();
   private final HeightMapSnapWiggler heightMapSnapWiggler;

   // Use this by default
   public FootstepSnapAndWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame,
                                 DefaultFootstepPlannerParametersReadOnly parameters,
                                 EnvironmentHandler environmentHandler)
   {
      this(footPolygonsInSoleFrame, parameters, null, environmentHandler, null, null);
   }

   // Call this constructor only for testing
   public FootstepSnapAndWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame,
                                 DefaultFootstepPlannerParametersReadOnly parameters,
                                 TickAndUpdatable tickAndUpdatable,
                                 EnvironmentHandler environmentHandler,
                                 YoGraphicsListRegistry graphicsListRegistry,
                                 YoRegistry parentRegistry)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
      this.parameters = parameters;
      this.heightMapSnapWiggler = new HeightMapSnapWiggler(footPolygonsInSoleFrame, wiggleParameters);
      this.environmentHandler = environmentHandler;
   }

   public void setFlatGroundHeight(double flatGroundHeight)
   {
      this.flatGroundHeight = flatGroundHeight;
   }

   public void initialize()
   {
      updateWiggleParameters(wiggleParameters, parameters);
   }

   public void clearSnapData()
   {
      snappedFootsteps.forEach(DiscreteFootstep::clearSnapData);
      snappedFootsteps.clear();
      manuallySnappedFootsteps.clear();
   }

   public FootstepSnapData snapFootstep(DiscreteFootstep footstep)
   {
      return snapFootstep(footstep, null, false);
   }

   @Override
   public FootstepSnapData snapFootstep(DiscreteFootstep footstep, DiscreteFootstep stanceStep, boolean computeWiggleTransform)
   {
      if (footstep.hasSnapData())
      {
         FootstepSnapData snapData = footstep.getSnapData();
         if (snapData.getSnapTransform().containsNaN())
         {
            return snapData;
         }
         else if (snapData.getWiggleTransformInWorld().containsNaN() && computeWiggleTransform)
         {
            computeWiggleTransform(footstep, stanceStep, snapData);
            updateTheCroppedFoothold(footstep);
         }

         return snapData;
      }
      else if (manuallySnappedFootsteps.containsKey(footstep))
      {
         FootstepSnapData snapData = manuallySnappedFootsteps.get(footstep);
         footstep.setSnapData(snapData);
         return snapData;
      }
      else
      {
         if (environmentHandler.flatGroundMode())
         {
            return FootstepSnapData.identityData(flatGroundHeight);
         }
         else
         {
            FootstepSnapData snapData = computeSnapTransform(footstep, stanceStep);
            footstep.setSnapData(snapData);
            snappedFootsteps.add(footstep);

            if (snapData.getSnapTransform().containsNaN())
            {
               return snapData;
            }
            else if (computeWiggleTransform)
            {
               computeWiggleTransform(footstep, stanceStep, snapData);
            }

            return snapData;
         }
      }
   }

   protected FootstepSnapData computeSnapTransform(DiscreteFootstep footstepToSnap, DiscreteFootstep stanceStep)
   {
      return heightMapSnapper.computeSnapData(footstepToSnap, footPolygonsInSoleFrame.get(footstepToSnap.getRobotSide()), environmentHandler);
   }

   /**
    * Can manually add snap data for a footstep to bypass the snapper.
    */
   public void addSnapData(DiscreteFootstep footstep, FootstepSnapData snapData)
   {
      footstep.setSnapData(snapData);
      snappedFootsteps.add(footstep);
      manuallySnappedFootsteps.put(footstep, snapData);
   }

   protected void computeWiggleTransform(DiscreteFootstep footstepToWiggle, DiscreteFootstep stanceStep, FootstepSnapData snapData)
   {
      heightMapSnapWiggler.computeWiggleTransform(footstepToWiggle, environmentHandler, snapData);

      if (stanceStep != null && stanceStep.hasSnapData())
      {
         FootstepSnapData stanceStepSnapData = stanceStep.getSnapData();

         // check for overlap
         boolean overlapDetected = stepsAreTooClose(footstepToWiggle, snapData, stanceStep, stanceStepSnapData);
         if (overlapDetected)
         {
            snapData.getWiggleTransformInWorld().setIdentity();
         }

         // check for overlap after this steps wiggle is removed. if still overlapping, remove wiggle on stance step
         overlapDetected = stepsAreTooClose(footstepToWiggle, snapData, stanceStep, stanceStepSnapData);
         if (overlapDetected)
         {
            stanceStepSnapData.getWiggleTransformInWorld().setIdentity();
         }
      }
   }

   protected void updateTheCroppedFoothold(DiscreteFootstep footstepToCrop)
   {
      List<Point3DBasics> footPointsInEnvironment = new ArrayList<>();
      FootstepSnapData snapData = footstepToCrop.getSnapData();

      ConvexPolygon2D footPolygonInFootFrame = new ConvexPolygon2D(snapData.getCroppedFoothold());
      if (footPolygonInFootFrame.getNumberOfVertices() != 4)
      {
         footPolygonInFootFrame.set(footPolygonsInSoleFrame.get(footstepToCrop.getRobotSide()));
      }

      // Get the transform from the sole frame to the desired foot pose.
      RigidBodyTransform transformToFootPose = new RigidBodyTransform();
      RigidBodyTransform snapAndWiggleTransform = new RigidBodyTransform();

      DiscreteFootstepTools.getStepTransform(footstepToCrop, transformToFootPose);
      snapData.packSnapAndWiggleTransform(snapAndWiggleTransform);
      transformToFootPose.preMultiply(snapAndWiggleTransform);

      // Transform the foot polygon to its position in the world after the snap.
      ConvexPolygon2D snappedFootPolygonInWorld = new ConvexPolygon2D(footPolygonInFootFrame);
      snappedFootPolygonInWorld.applyTransform(transformToFootPose, false);

      heightMapSnapper.computeFootPointsInTheEnvironment(snappedFootPolygonInWorld,
                                                         environmentHandler.getTerrainMapData(),
                                                         parameters.getHeightMapSnapThreshold(),
                                                         parameters.getMinSurfaceIncline(),
                                                         -Double.MAX_VALUE,
                                                         footPointsInEnvironment);

      List<Point2D> footPointsInFoot = footPointsInEnvironment.stream().map(point ->
                                                                            {
                                                                               Point3D transformedPoint = new Point3D(point);
                                                                               transformedPoint.applyInverseTransform(transformToFootPose);
                                                                               return new Point2D(transformedPoint);
                                                                            }).toList();

      ConvexPolygon2D croppedFoothold = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPointsInFoot));

      if (croppedFoothold.getNumberOfVertices() < 4)
      {
         // something in gift wrapping the polygon failed. Fall back to the original polygon.
         croppedFoothold.set(footPolygonInFootFrame);
      }
      else
      {
         // Check that all points are inside the polygon. If not, gift wrapping failed, and we need to fall back to the original polygon.
         for (Point2DBasics footPoint : footPointsInFoot)
         {
            if (!croppedFoothold.isPointInside(footPoint, 1e-3))
            {
               croppedFoothold.set(footPolygonInFootFrame);
               break;
            }
         }
      }

      // Reduce the foothold down to 4 vertices.
      if (croppedFoothold.getNumberOfVertices() > 4)
         ConvexPolygonTools.limitVerticesConservative(croppedFoothold, 4);

      if (croppedFoothold.getArea() < 0.05 * footPolygonInFootFrame.getArea())
      {
         // something about the foot polygon is too small. Fall back to the original polygon.
         croppedFoothold.set(footPolygonInFootFrame);
      }

      // If this cropped foothold is almost the same as the original, don't bother storing it, and clear the stored one.
      if (footPolygonsInSoleFrame.get(footstepToCrop.getRobotSide()).getArea() - croppedFoothold.getArea() > 1e-3)
         snapData.getCroppedFoothold().set(croppedFoothold);
      else
         snapData.getCroppedFoothold().clear();
   }

   private final RigidBodyTransform transform1 = new RigidBodyTransform();
   private final RigidBodyTransform transform2 = new RigidBodyTransform();
   private final ConvexPolygon2D polygon1 = new ConvexPolygon2D();
   private final ConvexPolygon2D polygon2 = new ConvexPolygon2D();

   /**
    * Extracted to method for testing purposes
    */
   protected boolean stepsAreTooClose(DiscreteFootstep step1, FootstepSnapData snapData1, DiscreteFootstep step2, FootstepSnapData snapData2)
   {
      DiscreteFootstepTools.getFootPolygon(step1, footPolygonsInSoleFrame.get(step1.getRobotSide()), polygon1);
      DiscreteFootstepTools.getFootPolygon(step2, footPolygonsInSoleFrame.get(step2.getRobotSide()), polygon2);

      snapData1.packSnapAndWiggleTransform(transform1);
      snapData2.packSnapAndWiggleTransform(transform2);

      polygon1.applyTransform(transform1, false);
      polygon2.applyTransform(transform2, false);

      boolean intersection = StepConstraintPolygonTools.arePolygonsIntersecting(polygon1, polygon2);
      if (intersection)
      {
         return true;
      }

      double distance = StepConstraintPolygonTools.distanceBetweenPolygons(polygon1, polygon2);
      return distance < parameters.getMinClearanceFromStance();
   }

   private static void updateWiggleParameters(WiggleParameters wiggleParameters, DefaultFootstepPlannerParametersReadOnly parameters)
   {
      wiggleParameters.deltaInside = parameters.getWiggleInsideDeltaTarget();
      wiggleParameters.maxX = parameters.getMaxXYWiggleDistance();
      wiggleParameters.minX = -parameters.getMaxXYWiggleDistance();
      wiggleParameters.maxY = parameters.getMaxXYWiggleDistance();
      wiggleParameters.minY = -parameters.getMaxXYWiggleDistance();
      wiggleParameters.maxYaw = parameters.getMaxYawWiggle();
      wiggleParameters.minYaw = -parameters.getMaxYawWiggle();
   }

   /**
    * Clears snapper history
    */
   public void reset()
   {
      clearSnapData();
   }

   public static void main(String[] args)
   {
      List<Point2D> points = new ArrayList<>();
      points.add(new Point2D(0.10749938776083733, 0.02999771516261329));  // 0
      points.add(new Point2D(0.06449944944584798, 0.033497945367126664)); // 1
      points.add(new Point2D(0.021499511130858618, 0.03699817557164005)); // 2
      points.add(new Point2D(-0.021500427184130748, 0.04049840577615344)); // 3
      points.add(new Point2D(-0.06450036549912008, 0.04399863598066683)); // 4
      points.add(new Point2D(-0.10750030381410944, 0.04749886618518021)); // 5
      points.add(new Point2D(0.10749948003264906, 0.017998059515111544)); // 6
      points.add(new Point2D(0.0644995524827044, 0.02009832989408306)); // 7
      points.add(new Point2D(0.02149962493275975, 0.022198600273054574)); // 8
      points.add(new Point2D(-0.021500302617184913, 0.024298870652026083)); //9
      points.add(new Point2D(-0.06450023016712958, 0.026399141030997587)); //10
      points.add(new Point2D(-0.1075001577170742, 0.02849941140996912)); //11
      points.add(new Point2D(0.10749957230446079, 0.0059984038676098046)); //12
      points.add(new Point2D(0.06449965551956086, 0.006698714421039443)); //13
      points.add(new Point2D(0.021499738734660895, 0.007399024974469082)); //14
      points.add(new Point2D(-0.021500178050239065, 0.008099335527898733)); //15
      points.add(new Point2D(-0.06450009483513901, 0.008799646081328378)); //16
      points.add(new Point2D(-0.10750001162003894, 0.00949995663475803)); //17
      points.add(new Point2D(0.10749966457627252, -0.006001251779891947)); //18
      points.add(new Point2D(0.06449975855641729, -0.006700901052004176)); //19
      points.add(new Point2D(0.021499852536562034, -0.0074005503241163974)); //20

      points.add(new Point2D(-0.021500053483293216, -0.008100199596228616)); //21
      points.add(new Point2D(-0.06449995950314846, -0.008799848868340851)); //22
      points.add(new Point2D(-0.10749986552300368, -0.009499498140453073)); //23
      points.add(new Point2D(0.10749975684808429, -0.018000907427393683)); //24
      points.add(new Point2D(0.06449986159327373, -0.020100516525047785)); //25
      points.add(new Point2D(0.02149996633846318, -0.022200125622701867)); //26
      points.add(new Point2D(-0.021499928916347368, -0.02429973472035595)); //27
      points.add(new Point2D(-0.06449982417115792, -0.026399343818010065)); //28
      points.add(new Point2D(-0.10749971942596843, -0.02849895291566415)); //29

      points.add(new Point2D(0.10749984911989602, -0.030000563074895428)); //30
      points.add(new Point2D(0.06449996463013016, -0.03350013199809138)); //31
      points.add(new Point2D(0.021500080140364322, -0.03699970092128736)); //32
      points.add(new Point2D(-0.021499804349401512, -0.040499269844483314)); //33
      points.add(new Point2D(-0.06449968883916736, -0.04399883876767927)); //34
      points.add(new Point2D(-0.10749957332893317, -0.04749840769087523)); //35

      ConvexPolygon2D polygon2D = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(points));

      for (Point2D point : points)
      {
         assert (polygon2D.signedDistance(point) < 1e-3);
      }
   }
}
