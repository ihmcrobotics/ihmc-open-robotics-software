//package us.ihmc.rdx.ui.vr;
//
//import imgui.ImGui;
//import imgui.type.ImBoolean;
//import imgui.type.ImInt;
//import imgui.type.ImString;
//import us.ihmc.avatar.stereoVision.StereoVision;
//import us.ihmc.euclid.referenceFrame.ReferenceFrame;
//import us.ihmc.perception.ImageDimensions;
//import us.ihmc.rdx.imgui.ImGuiFlashingText;
//import us.ihmc.rdx.imgui.ImGuiTools;
//import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
//import us.ihmc.rdx.perception.RDXDualBlackflySphericalProjection;
//import us.ihmc.robotics.robotSide.RobotSide;
//import us.ihmc.robotics.robotSide.SideDependentList;
//
//import java.util.concurrent.atomic.AtomicInteger;
//
//public class RDXVRStereoVision
//{
//   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
//
//   private final ImBoolean enabled = new ImBoolean(false);
//   private final RDXDualBlackflySphericalProjection dualBlackflySphericalProjection;
//   private final ImGuiFlashingText reconnectingText = new ImGuiFlashingText(ImGuiTools.DARK_ORANGE);
//   private final ImGuiFlashingText leftConnectedStatusCircle = new ImGuiFlashingText(ImGuiTools.DARK_GREEN);
//   private final ImGuiFlashingText rightConnectedStatusCircle = new ImGuiFlashingText(ImGuiTools.DARK_GREEN);
//   private final ImString leftConnectionAddress = new ImString();
//   private final ImString rightConnectionAddress = new ImString();
//
//   private SideDependentList<ImBoolean> autoOffsets = new SideDependentList<>();
//
//   public RDXVRStereoVision(ReferenceFrame robotzUpFrame)
//   {
//      dualBlackflySphericalProjection = new RDXDualBlackflySphericalProjection(robotzUpFrame);
//
//      autoOffsets.put(RobotSide.LEFT, new ImBoolean(true));
//      autoOffsets.put(RobotSide.RIGHT, new ImBoolean(true));
//   }
//
//   public boolean isEnabled()
//   {
//      return enabled.get();
//   }
//
//   private ImageDimensions getLastImageDimensions(RobotSide side)
//   {
//      return dualBlackflySphericalProjection.getDualBlackflyUDPReceiver().getImageDimensions().get(side);
//   }
//
//   private AtomicInteger getLastFrameNumber(RobotSide side)
//   {
//      return dualBlackflySphericalProjection.getDualBlackflyUDPReceiver().getFrameNumbers().get(side);
//   }
//
//   public void renderProjection()
//   {
//      if (enabled.get())
//         dualBlackflySphericalProjection.render();
//   }
//
//   public void renderControls()
//   {
//      if (enabled.get() && !dualBlackflySphericalProjection.isConnectingOrConnected())
//         dualBlackflySphericalProjection.enable();
//
//      if (ImGui.checkbox(labels.get("Stereo vision enabled"), enabled))
//      {
//         if (!enabled.get())
//         {
//            dualBlackflySphericalProjection.disable();
//         }
//      }
//
//      if (dualBlackflySphericalProjection.isConnected())
//      {
//         ImGui.textColored(ImGuiTools.DARK_GREEN, "Connected");
//
//         double leftFrequency = dualBlackflySphericalProjection.getDualBlackflyUDPReceiver().getFrequencyCalculators().get(RobotSide.LEFT).getFrequency();
//         double rightFrequency = dualBlackflySphericalProjection.getDualBlackflyUDPReceiver().getFrequencyCalculators().get(RobotSide.RIGHT).getFrequency();
//
//         leftConnectedStatusCircle.setPeriod(1000 / leftFrequency);
//         rightConnectedStatusCircle.setPeriod(1000 / rightFrequency);
//
//         for (RobotSide side : RobotSide.values)
//         {
//            ImGui.sameLine();
//            leftConnectedStatusCircle.renderText("(" + side.getSideNameFirstLetter() + ")", leftFrequency != 0);
//            ImageDimensions imageDimensions = getLastImageDimensions(side);
//            int width = imageDimensions.getImageWidth();
//            int height = imageDimensions.getImageHeight();
//            AtomicInteger frameNumber = getLastFrameNumber(side);
//            ImGuiTools.previousWidgetTooltip("Resolution: " + width + "x" + height + " \nFrame number: " + frameNumber.get());
//         }
//      }
//      else if (dualBlackflySphericalProjection.isReconnecting())
//      {
//         reconnectingText.renderText("Reconnecting...", true);
//      }
//      else
//      {
//         ImGui.textColored(ImGuiTools.DARK_RED, "Not connected");
//      }
//
//      if (ImGui.collapsingHeader(labels.get("Connection and image controls")))
//      {
//         ImGui.text("Connection details (UDP ports [" + StereoVision.LEFT_UDP_PORT + ", " + StereoVision.RIGHT_UDP_PORT + "])");
//
//         boolean updatedAddress = false;
//
//         if (leftConnectionAddress.isEmpty())
//         {
//            leftConnectionAddress.set(dualBlackflySphericalProjection.getDualBlackflyUDPReceiver().getAddresses().get(RobotSide.LEFT));
//         }
//
//         if (rightConnectionAddress.isEmpty())
//         {
//            rightConnectionAddress.set(dualBlackflySphericalProjection.getDualBlackflyUDPReceiver().getAddresses().get(RobotSide.RIGHT));
//         }
//
//         if (ImGui.inputText(labels.get("(L) address"), leftConnectionAddress))
//         {
//            dualBlackflySphericalProjection.getDualBlackflyUDPReceiver().setAddress(RobotSide.LEFT, leftConnectionAddress.get());
//            updatedAddress = true;
//         }
//         if (ImGui.inputText(labels.get("(R) address"), rightConnectionAddress))
//         {
//            dualBlackflySphericalProjection.getDualBlackflyUDPReceiver().setAddress(RobotSide.RIGHT, rightConnectionAddress.get());
//            updatedAddress = true;
//         }
//
//         if (updatedAddress)
//         {
//            enabled.set(false);
//            dualBlackflySphericalProjection.disable();
//         }
//
//         for (RobotSide side : RobotSide.values)
//         {
//            ImGui.separator();
//            ImGui.text(side.getCamelCaseNameForMiddleOfExpression() + " camera image control");
//            ImGui.inputInt(labels.get("(" + side.getSideNameFirstLetter() + ") width"), new ImInt());
//            ImGui.inputInt(labels.get("(" + side.getSideNameFirstLetter() + ") height"), new ImInt());
//            ImGui.checkbox(labels.get("(" + side.getSideNameFirstLetter() + ") auto offsets"), autoOffsets.get(side));
//            ImGuiTools.previousWidgetTooltip("Automatically center the image based on the dimensions");
//            if (autoOffsets.get(side).get())
//               ImGui.beginDisabled();
//            ImGui.inputInt(labels.get("(" + side.getSideNameFirstLetter() + ") offset X"), new ImInt());
//            ImGui.inputInt(labels.get("(" + side.getSideNameFirstLetter() + ") offset Y"), new ImInt());
//            if (autoOffsets.get(side).get())
//               ImGui.endDisabled();
//         }
//      }
//
//      if (ImGui.collapsingHeader(labels.get("Projection sphere controls")))
//      {
//         dualBlackflySphericalProjection.renderControls();
//      }
//   }
//
//   public RDXDualBlackflySphericalProjection getDualBlackflySphericalProjection()
//   {
//      return dualBlackflySphericalProjection;
//   }
//}
