package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.commonWalkingControlModules.polygonWiggling.GradientDescentStepConstraintInput;
import us.ihmc.commonWalkingControlModules.polygonWiggling.GradientDescentStepConstraintSolver;
import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.HashMap;

public class FootstepNodeSnapAndWiggler implements FootstepNodeSnapperReadOnly
{
   private final SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame;
   private final FootstepPlannerParametersReadOnly parameters;

   private final GradientDescentStepConstraintSolver gradientDescentStepConstraintSolver = new GradientDescentStepConstraintSolver();
   private final WiggleParameters wiggleParameters = new WiggleParameters();
   private final PlanarRegion planarRegionToPack = new PlanarRegion();
   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();
   private final Cylinder3D legCollisionShape = new Cylinder3D();
   private final RigidBodyTransform legCollisionShapeToSoleTransform = new RigidBodyTransform();
   private final RigidBodyTransformGenerator transformGenerator = new RigidBodyTransformGenerator();
   private final GradientDescentStepConstraintInput gradientDescentStepConstraintInput = new GradientDescentStepConstraintInput();

   private final HashMap<FootstepNode, FootstepNodeSnapData> snapDataHolder = new HashMap<>();
   protected PlanarRegionsList planarRegionsList;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public FootstepNodeSnapAndWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame, FootstepPlannerParametersReadOnly parameters)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
      this.parameters = parameters;
   }

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
      snapDataHolder.clear();
   }

   public void initialize()
   {
      updateWiggleParameters(wiggleParameters, parameters);
   }

   private boolean flatGroundMode()
   {
      return planarRegionsList == null || planarRegionsList.isEmpty();
   }

   public FootstepNodeSnapData snapFootstepNode(FootstepNode footstepNode)
   {
      return snapFootstepNode(footstepNode, null,false);
   }

   @Override
   public FootstepNodeSnapData snapFootstepNode(FootstepNode footstepNode, FootstepNode stanceNode, boolean computeWiggleTransform)
   {
      if (snapDataHolder.containsKey(footstepNode))
      {
         FootstepNodeSnapData snapData = snapDataHolder.get(footstepNode);
         if (snapData.getSnapTransform().containsNaN())
         {
            return snapData;
         }
         else if (snapData.getWiggleTransformInWorld().containsNaN() && computeWiggleTransform)
         {
            computeWiggleTransform(footstepNode, stanceNode, snapData);
         }

         return snapData;
      }
      else if (flatGroundMode())
      {
         return FootstepNodeSnapData.identityData();
      }
      else
      {
         FootstepNodeSnapData snapData = computeSnapTransform(footstepNode, stanceNode);
         snapDataHolder.put(footstepNode, snapData);

         if (snapData.getSnapTransform().containsNaN())
         {
            return snapData;
         }
         else if (computeWiggleTransform)
         {
            computeWiggleTransform(footstepNode, stanceNode, snapData);
         }

         return snapData;
      }
   }

   /**
    * Can manually add snap data for a footstep node to bypass the snapper.
    */
   public void addSnapData(FootstepNode footstepNode, FootstepNodeSnapData snapData)
   {
      snapDataHolder.put(footstepNode, snapData);
   }

   protected FootstepNodeSnapData computeSnapTransform(FootstepNode footstepNode, FootstepNode stanceNode)
   {
      double maximumRegionHeightToConsider = getMaximumRegionHeightToConsider(stanceNode);
      FootstepNodeTools.getFootPolygon(footstepNode, footPolygonsInSoleFrame.get(footstepNode.getRobotSide()), footPolygon);

      RigidBodyTransform snapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon, planarRegionsList, maximumRegionHeightToConsider, planarRegionToPack);
      if (snapTransform == null)
      {
         return FootstepNodeSnapData.emptyData();
      }
      else
      {
         FootstepNodeSnapData snapData = new FootstepNodeSnapData(snapTransform);
         snapData.setRegionIndex(getIndex(planarRegionToPack, planarRegionsList));
         computeCroppedFoothold(footstepNode, snapData);
         return snapData;
      }
   }

   private static int getIndex(PlanarRegion planarRegion, PlanarRegionsList planarRegionsList)
   {
      double epsilon = 1e-7;
      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         if (planarRegionsList.getPlanarRegion(i).epsilonEquals(planarRegion, epsilon))
         {
            return i;
         }
      }

      return -1;
   }

   private double getMaximumRegionHeightToConsider(FootstepNode stanceNode)
   {
      if (stanceNode == null)
      {
         return Double.POSITIVE_INFINITY;
      }

      FootstepNodeSnapData snapData = snapDataHolder.get(stanceNode);
      if (snapData == null || snapData.getSnapTransform().containsNaN())
      {
         return Double.POSITIVE_INFINITY;
      }
      else
      {
         return parameters.getMaximumSnapHeight() + FootstepNodeTools.getSnappedNodeHeight(stanceNode, snapData.getSnapTransform());
      }
   }

   protected void computeWiggleTransform(FootstepNode footstepNode, FootstepNode stanceNode, FootstepNodeSnapData snapData)
   {
      int regionIndex = snapData.getRegionIndex();
      if (regionIndex == -1)
      {
         LogTools.warn("Could not find matching region id, unable to find wiggle transform. Region id = " + snapData.getRegionIndex());
         snapData.getWiggleTransformInWorld().setIdentity();
         return;
      }
      else
      {
         planarRegionToPack.set(planarRegionsList.getPlanarRegion(regionIndex));
      }

      FootstepNodeTools.getFootPolygon(footstepNode, footPolygonsInSoleFrame.get(footstepNode.getRobotSide()), footPolygon);
      tempTransform.set(snapData.getSnapTransform());
      tempTransform.preMultiply(planarRegionToPack.getTransformToLocal());
      ConvexPolygon2D footPolygonInSnapFrame = FootstepNodeSnappingTools.computeTransformedPolygon(footPolygon, tempTransform);

      RigidBodyTransform wiggleTransformInLocal;
      boolean concaveWigglerRequested = parameters.getEnableConcaveHullWiggler() && !planarRegionToPack.getConcaveHull().isEmpty();
      if (concaveWigglerRequested)
      {
         gradientDescentStepConstraintInput.clear();
         gradientDescentStepConstraintInput.setInitialStepPolygon(footPolygonInSnapFrame);
         gradientDescentStepConstraintInput.setWiggleParameters(wiggleParameters);
         gradientDescentStepConstraintInput.setPlanarRegion(planarRegionToPack);

         if (parameters.getEnableShinCollisionCheck())
         {
            RigidBodyTransform snappedNodeTransform = snapData.getSnappedNodeTransform(footstepNode);
            tempTransform.set(snappedNodeTransform);
            tempTransform.preMultiply(planarRegionToPack.getTransformToLocal());
            gradientDescentStepConstraintInput.setFootstepInRegionFrame(tempTransform);

            ConvexPolygon2D footPolygon = footPolygonsInSoleFrame.get(footstepNode.getRobotSide());
            double forwardPoint = footPolygon.getMaxX() + parameters.getShinToeClearance();
            double backwardPoint = footPolygon.getMinX() - parameters.getShinHeelClearance();
            double shinRadius = 0.5 * (forwardPoint - backwardPoint);
            double shinXOffset = 0.5 * (forwardPoint + backwardPoint);

            legCollisionShape.setSize(parameters.getShinLength(), shinRadius);
            transformGenerator.identity();
            transformGenerator.translate(shinXOffset, 0.0, parameters.getShinHeightOffset());
            transformGenerator.translate(0.0, 0.0, 0.5 * parameters.getShinLength());
            transformGenerator.getRigidyBodyTransform(legCollisionShapeToSoleTransform);
            gradientDescentStepConstraintSolver.setLegCollisionShape(legCollisionShape, legCollisionShapeToSoleTransform);

            gradientDescentStepConstraintInput.setPlanarRegionsList(planarRegionsList);
         }

         wiggleTransformInLocal = gradientDescentStepConstraintSolver.wigglePolygon(gradientDescentStepConstraintInput);
      }
      else
      {
         if (isConvexConstraintSatisfied(footPolygonInSnapFrame, planarRegionToPack, parameters.getWiggleInsideDelta()))
         {
            snapData.getWiggleTransformInWorld().setIdentity();
            return;
         }
         else
         {
            if ((wiggleTransformInLocal = wiggleIntoConvexHull(footPolygonInSnapFrame)) == null)
            {
               snapData.getWiggleTransformInWorld().setIdentity();
               return;
            }
         }
      }

      snapData.getWiggleTransformInWorld().set(planarRegionToPack.getTransformToLocal());
      snapData.getWiggleTransformInWorld().preMultiply(wiggleTransformInLocal);
      snapData.getWiggleTransformInWorld().preMultiply(planarRegionToPack.getTransformToWorld());

      if (stanceNode != null && snapDataHolder.containsKey(stanceNode))
      {
         FootstepNodeSnapData stanceNodeSnapData = snapDataHolder.get(stanceNode);

         // check for overlap
         boolean overlapDetected = stepsAreTooClose(footstepNode, snapData, stanceNode, stanceNodeSnapData);
         if (overlapDetected)
         {
            snapData.getWiggleTransformInWorld().setIdentity();
         }

         // check for overlap after this steps wiggle is removed. if still overlapping, remove wiggle on stance step
         overlapDetected = stepsAreTooClose(footstepNode, snapData, stanceNode, stanceNodeSnapData);
         if (overlapDetected)
         {
            stanceNodeSnapData.getWiggleTransformInWorld().setIdentity();
         }
      }

      computeCroppedFoothold(footstepNode, snapData);
   }

   /** Extracted to method for testing purposes */
   protected RigidBodyTransform wiggleIntoConvexHull(ConvexPolygon2D footPolygonInRegionFrame)
   {
      return PolygonWiggler.wigglePolygonIntoConvexHullOfRegion(footPolygonInRegionFrame, planarRegionToPack, wiggleParameters);
   }

   private final RigidBodyTransform transform1 = new RigidBodyTransform();
   private final RigidBodyTransform transform2 = new RigidBodyTransform();
   private final ConvexPolygon2D polyon1 = new ConvexPolygon2D();
   private final ConvexPolygon2D polyon2 = new ConvexPolygon2D();

   /** Extracted to method for testing purposes */
   protected boolean stepsAreTooClose(FootstepNode node1, FootstepNodeSnapData snapData1, FootstepNode node2, FootstepNodeSnapData snapData2)
   {
      FootstepNodeTools.getFootPolygon(node1, footPolygonsInSoleFrame.get(node1.getRobotSide()), polyon1);
      FootstepNodeTools.getFootPolygon(node2, footPolygonsInSoleFrame.get(node2.getRobotSide()), polyon2);

      snapData1.packSnapAndWiggleTransform(transform1);
      snapData2.packSnapAndWiggleTransform(transform2);

      polyon1.applyTransform(transform1, false);
      polyon2.applyTransform(transform2, false);

      boolean intersection = FootstepNodeTools.arePolygonsIntersecting(polyon1, polyon2);
      if (intersection)
      {
         return true;
      }

      double distance = FootstepNodeTools.distanceBetweenPolygons(polyon1, polyon2);
      return distance < parameters.getMinClearanceFromStance();
   }

   private final Vector4D polygonVertexToTransform = new Vector4D();
   private final ArrayList<ConvexPolygon2D> polygonIntersections = new ArrayList<>();

   protected void computeCroppedFoothold(FootstepNode footstepNode, FootstepNodeSnapData snapData)
   {
      if (flatGroundMode())
      {
         snapData.getCroppedFoothold().clearAndUpdate();
         return;
      }

      FootstepNodeTools.getFootPolygon(footstepNode, footPolygonsInSoleFrame.get(footstepNode.getRobotSide()), footPolygon);

      snapData.packSnapAndWiggleTransform(tempTransform);
      ConvexPolygon2D snappedPolygonInWorld = FootstepNodeSnappingTools.computeTransformedPolygon(footPolygon, tempTransform);
      ConvexPolygon2D croppedFootPolygon = FootstepNodeSnappingTools.computeRegionIntersection(planarRegionToPack, snappedPolygonInWorld);

      if (!croppedFootPolygon.isEmpty())
      {
         FootstepNodeSnappingTools.changeFromPlanarRegionToSoleFrame(planarRegionToPack, footstepNode, tempTransform, croppedFootPolygon);
         snapData.getCroppedFoothold().set(croppedFootPolygon);
      }

      checkForNearbyContact(footstepNode, snapData, snappedPolygonInWorld, croppedFootPolygon);
   }

   private void checkForNearbyContact(FootstepNode footstepNode,
                                      FootstepNodeSnapData snapData,
                                      ConvexPolygon2D snappedPolygonInWorld,
                                      ConvexPolygon2D croppedFootPolygon)
   {
      boolean checkForNearbyContact = parameters.getDistanceEpsilonToBridgeRegions() > 0.0;
      if (!checkForNearbyContact)
      {
         return;
      }

      boolean fullFoothold = croppedFootPolygon.isEmpty() || croppedFootPolygon.getArea() / footPolygonsInSoleFrame.get(footstepNode.getRobotSide()).getArea() > 0.99;
      if (fullFoothold)
      {
         return;
      }

      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion candidateRegion = planarRegionsList.getPlanarRegion(i);

         if (candidateRegion.epsilonEquals(planarRegionToPack, 1e-3))
         {
            continue;
         }

         if (candidateRegion.isVertical())
         {
            continue;
         }

         if (candidateRegion.isPolygonIntersecting(snappedPolygonInWorld))
         {
            boolean appendToFoothold = true;
            for (int j = 0; j < snappedPolygonInWorld.getNumberOfVertices(); j++)
            {
               FootstepNodeSnappingTools.transformPolygonVertex(snappedPolygonInWorld.getVertex(j), polygonVertexToTransform, tempTransform);
               double snappedVertexZ = polygonVertexToTransform.getZ();
               double planeZ = candidateRegion.getPlaneZGivenXY(polygonVertexToTransform.getX(), polygonVertexToTransform.getY());

               if (Math.abs(snappedVertexZ - planeZ) > parameters.getDistanceEpsilonToBridgeRegions())
               {
                  appendToFoothold = false;
                  break;
               }
            }

            if (appendToFoothold)
            {
               polygonIntersections.clear();
               candidateRegion.getPolygonIntersectionsWhenProjectedVertically(snappedPolygonInWorld, polygonIntersections);

               for (int j = 0; j < polygonIntersections.size(); j++)
               {
                  ConvexPolygon2D polygonIntersection = polygonIntersections.get(j);
                  FootstepNodeSnappingTools.changeFromPlanarRegionToSoleFrame(candidateRegion, footstepNode, tempTransform, polygonIntersection);
                  croppedFootPolygon.addVertices(polygonIntersection);
               }

               croppedFootPolygon.update();
               snapData.getCroppedFoothold().set(croppedFootPolygon);
            }
         }
      }
   }

   private static void updateWiggleParameters(WiggleParameters wiggleParameters, FootstepPlannerParametersReadOnly parameters)
   {
      wiggleParameters.deltaInside = parameters.getWiggleInsideDelta();
      wiggleParameters.maxX = parameters.getMaximumXYWiggleDistance();
      wiggleParameters.minX = -parameters.getMaximumXYWiggleDistance();
      wiggleParameters.maxY = parameters.getMaximumXYWiggleDistance();
      wiggleParameters.minY = -parameters.getMaximumXYWiggleDistance();
      wiggleParameters.maxYaw = parameters.getMaximumYawWiggle();
      wiggleParameters.minYaw = -parameters.getMaximumYawWiggle();
   }

   private static boolean isConvexConstraintSatisfied(ConvexPolygon2DReadOnly footPolygon, PlanarRegion planarRegion, double wiggleInsideDelta)
   {
      for (int i = 0; i < footPolygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = footPolygon.getVertex(i);
         if (planarRegion.getConvexHull().signedDistance(vertex) > - wiggleInsideDelta)
         {
            return false;
         }
      }

      return true;
   }

   /**
    * Clears snapper history
    */
   public void reset()
   {
      snapDataHolder.clear();
   }
}
