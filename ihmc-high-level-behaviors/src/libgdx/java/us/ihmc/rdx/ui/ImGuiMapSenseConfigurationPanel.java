package us.ihmc.rdx.ui;

import controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage;
import perception_msgs.msg.dds.PolygonizerParametersStringMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import std_msgs.msg.dds.Float64;
import us.ihmc.avatar.sensors.realsense.DelayFixedPlanarRegionsSubscription;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.publisher.RosMapsenseConfigurationPublisher;

public class ImGuiMapSenseConfigurationPanel
{
   private static final String WINDOW_NAME = "MapSense Configuration";

   private final ImFloat planarRegionsDelayOffset = new ImFloat((float) DelayFixedPlanarRegionsSubscription.INITIAL_DELAY_OFFSET);

   private final ImFloat edgeLengthTresholdSlider = new ImFloat(0.224f);
   private final ImFloat triangulationToleranceSlider = new ImFloat(0.0f);
   private final ImInt maxNumberOfIterationsSlider = new ImInt(5000);
   private final ImBoolean allowSplittingConcaveHullChecked = new ImBoolean(false);
   private final ImBoolean removeAllTrianglesWithTwoBorderEdgesChecked = new ImBoolean(false);

   private final ImFloat concaveHullThresholdSlider = new ImFloat(0.15f);
   private final ImFloat shallowAngleThresholdSlider = new ImFloat((float) Math.toRadians(1.0));
   private final ImFloat peakAngleThresholdSlider = new ImFloat((float) Math.toRadians(170.0));
   private final ImFloat lengthThresholdSlider = new ImFloat(0.05f);
   private final ImFloat depthThresholdSlider = new ImFloat(0.10f);
   private final ImInt minNumberOfNodesSlider = new ImInt(10);
   private final ImBoolean cutNarrowPassageChecked = new ImBoolean(true);

   private final ImFloat mergeDistanceThresholdSlider = new ImFloat(0.016f);
   private final ImFloat mergeAngularThresholdSlider = new ImFloat(0.82f);
   private final ImFloat regionGrowthFactorSlider = new ImFloat(0.01f);
   private final ImInt gaussianSizeSlider = new ImInt(6);
   private final ImInt gaussianSigmaSlider = new ImInt(20);

   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private final ROS2PublisherMap ros2Publisher;
   private final RosMapsenseConfigurationPublisher mapSenseConfigurationPublisher;

   public ImGuiMapSenseConfigurationPanel(RosNodeInterface ros1Node, ROS2NodeInterface ros2Node)
   {
      ros2Publisher = new ROS2PublisherMap(ros2Node);

      mapSenseConfigurationPublisher = new RosMapsenseConfigurationPublisher();
      ros1Node.attachPublisher(RosTools.MAPSENSE_CONFIGURATION, mapSenseConfigurationPublisher);
   }

   public void render()
   {
      if (ImGui.sliderFloat("Planar region delay offset", planarRegionsDelayOffset.getData(), -0.75f, 0.75f))
      {
         Float64 message = new Float64();
         message.setData(planarRegionsDelayOffset.get());
         ros2Publisher.publish(PerceptionAPI.MAPSENSE_REGIONS_DELAY_OFFSET, message);
      }

      boolean concaveHullFactoryParametersChanged = false;
      concaveHullFactoryParametersChanged |= ImGui.sliderFloat("Edge Length Threshold", edgeLengthTresholdSlider.getData(), 0, 0.5f);
      concaveHullFactoryParameters.setEdgeLengthThreshold(edgeLengthTresholdSlider.get());
      concaveHullFactoryParametersChanged |= ImGui.sliderFloat("Triangulation Tolerance", triangulationToleranceSlider.getData(), 0, 1);
      concaveHullFactoryParameters.setTriangulationTolerance(triangulationToleranceSlider.get());
      concaveHullFactoryParametersChanged |= ImGui.sliderInt("Max Number of Iterations", maxNumberOfIterationsSlider.getData(), 2000, 6000);
      concaveHullFactoryParameters.setMaxNumberOfIterations(maxNumberOfIterationsSlider.get());
      concaveHullFactoryParametersChanged |= ImGui.checkbox("Remove Degenerate Triangles", removeAllTrianglesWithTwoBorderEdgesChecked);
      concaveHullFactoryParameters.setRemoveAllTrianglesWithTwoBorderEdges(removeAllTrianglesWithTwoBorderEdgesChecked.get());
      concaveHullFactoryParametersChanged |= ImGui.checkbox("Split Concave Hull", allowSplittingConcaveHullChecked);
      concaveHullFactoryParameters.setAllowSplittingConcaveHull(allowSplittingConcaveHullChecked.get());
      if (concaveHullFactoryParametersChanged)
      {
         ConcaveHullFactoryParametersStringMessage message = new ConcaveHullFactoryParametersStringMessage();
         message.getParameters().add(concaveHullFactoryParameters.toString());
         ros2Publisher.publish(PerceptionAPI.CONCAVE_HULL_FACTORY_PARAMETERS, message);
      }

      boolean polygonizerParametersChanged = false;
      polygonizerParametersChanged |= ImGui.sliderFloat("Concave Hull Threshold", concaveHullThresholdSlider.getData(), 0, 1);
      polygonizerParameters.setConcaveHullThreshold(concaveHullThresholdSlider.get());
      polygonizerParametersChanged |= ImGui.sliderFloat("Shallow Angle Threshold", shallowAngleThresholdSlider.getData(), 0, 2.0f * (float) Math.PI);
      polygonizerParameters.setShallowAngleThreshold(shallowAngleThresholdSlider.get());
      polygonizerParametersChanged |= ImGui.sliderFloat("Peak Angle Threshold", peakAngleThresholdSlider.getData(), 0, 2.0f * (float) Math.PI);
      polygonizerParameters.setPeakAngleThreshold(peakAngleThresholdSlider.get());
      polygonizerParametersChanged |= ImGui.sliderFloat("Length Threshold", lengthThresholdSlider.getData(), 0, 1);
      polygonizerParameters.setLengthThreshold(lengthThresholdSlider.get());
      polygonizerParametersChanged |= ImGui.sliderFloat("Depth Threshold", depthThresholdSlider.getData(), 0, 1);
      polygonizerParameters.setDepthThreshold(depthThresholdSlider.get());
      polygonizerParametersChanged |= ImGui.sliderInt("Min Number of Nodes", minNumberOfNodesSlider.getData(), 0, 100);
      polygonizerParameters.setMinNumberOfNodes(minNumberOfNodesSlider.get());
      polygonizerParametersChanged |= ImGui.checkbox("Cut Narrow Passage", cutNarrowPassageChecked);
      polygonizerParameters.setCutNarrowPassage(cutNarrowPassageChecked.get());
      if (polygonizerParametersChanged)
      {
         PolygonizerParametersStringMessage message = new PolygonizerParametersStringMessage();
         message.getParameters().add(polygonizerParameters.toString());
         ros2Publisher.publish(PerceptionAPI.POLYGONIZER_PARAMETERS, message);
      }

      boolean mapSenseParamsChanged = false;
      mapSenseParamsChanged |= ImGui.sliderFloat("Merge Distance Threshold", mergeDistanceThresholdSlider.getData(), 0.0f, 0.1f);
      mapSenseParamsChanged |= ImGui.sliderFloat("Merge Angular Threshold", mergeAngularThresholdSlider.getData(), 0.0f, 1.0f);
      mapSenseParamsChanged |= ImGui.sliderFloat("Region Growth Factor", regionGrowthFactorSlider.getData(), 0.001f, 0.1f);
      mapSenseParamsChanged |= ImGui.sliderInt("Gaussian Size", gaussianSizeSlider.getData(), 1, 8);
      mapSenseParamsChanged |= ImGui.sliderInt("Gaussian Sigma", gaussianSigmaSlider.getData(), 1, 20);
      if (mapSenseParamsChanged)
      {
         mapSenseConfigurationPublisher.publish((byte) 0,
                                                (byte) 0,
                                                mergeAngularThresholdSlider.get(),
                                                mergeDistanceThresholdSlider.get(),
                                                regionGrowthFactorSlider.get(),
                                                (byte) gaussianSizeSlider.get(),
                                                (byte) gaussianSigmaSlider.get());

         LogTools.info("Publishing Values Now");
      }

      /*
         ----ImGui Read-only MapSense Parameters
         filterPatchSize
         packPatchSize
         mergePatchSize
      * */
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }
}
