package us.ihmc.gdx.perception;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import org.asynchttpclient.DefaultAsyncHttpClient;
import org.asynchttpclient.Dsl;
import org.asynchttpclient.ListenableFuture;
import org.asynchttpclient.netty.ws.NettyWebSocket;
import org.asynchttpclient.ws.WebSocket;
import org.asynchttpclient.ws.WebSocketListener;
import org.asynchttpclient.ws.WebSocketUpgradeHandler;
import org.freedesktop.gstreamer.*;
import org.freedesktop.gstreamer.elements.DecodeBin;
import org.freedesktop.gstreamer.webrtc.WebRTCBin;
import org.freedesktop.gstreamer.webrtc.WebRTCSDPType;
import org.freedesktop.gstreamer.webrtc.WebRTCSessionDescription;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;

import java.io.IOException;

/**
 * Requires th following installed on the system:
 * - gstreamer
 * - gst-plugins-bad (includes the required webrtc and webrtcbin)
 * - libsoup
 *
 * Visit https://webrtc.nirbheek.in/ and pass the "Our id " given to this program.
 */
public class GDXWebcamGStreamerWebRTCSender
{
   private final String SERVER_URL = System.getProperty("server.url", "wss://localhost:8443");
   private final String SESSION_ID = System.getProperty("session.id", "0000");
   private final ObjectMapper mapper = new ObjectMapper();
   private DefaultAsyncHttpClient nettyHTTPClient;
   private NettyWebSocket websocket;
   private WebRTCBin webRTCBin;
   private Pipeline pipeline;

   private static final String PIPELINE_DESCRIPTION
         = "videotestsrc is-live=true pattern=ball"
      + " ! videoconvert"
      + " ! queue"
      + " ! vp8enc deadline=1"
      + " ! rtpvp8pay"
      + " ! queue"
      + " ! application/x-rtp,media=video,encoding-name=VP8,payload=97"
      + " ! webrtcbin. audiotestsrc is-live=true wave=sine"
      + " ! audioconvert"
      + " ! audioresample"
      + " ! queue"
      + " ! opusenc"
      + " ! rtpopuspay"
      + " ! queue"
      + " ! application/x-rtp,media=audio,encoding-name=OPUS,payload=96"
      + " ! webrtcbin. webrtcbin name=webrtcbin bundle-policy=max-bundle stun-server=stun://stun.l.google.com:19302 ";

   public GDXWebcamGStreamerWebRTCSender()
   {
      Gst.init(Version.of(1, 16));

      pipeline = (Pipeline) Gst.parseLaunch(PIPELINE_DESCRIPTION);
      setupPipeLogging(pipeline);

      webRTCBin = (WebRTCBin) pipeline.getElementByName("webrtcbin");

      webRTCBin.connect(onNegotiationNeeded);
      webRTCBin.connect(onIceCandidate);
      webRTCBin.connect(onIncomingStream);

      nettyHTTPClient = (DefaultAsyncHttpClient) Dsl.asyncHttpClient();
      WebSocketUpgradeHandler handler = new WebSocketUpgradeHandler.Builder().addWebSocketListener(webSocketListener).build();
      ListenableFuture<NettyWebSocket> execute = nettyHTTPClient.prepareGet(SERVER_URL).execute(handler);
      websocket = ExceptionTools.handle(() -> execute.get(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

      Gst.main();
   }

   private void endCall()
   {
      pipeline.setState(State.NULL);
      nettyHTTPClient.close();
      Gst.quit();
   }

   private final WebSocketListener webSocketListener = new WebSocketListener()
   {
      @Override
      public void onOpen(WebSocket websocket)
      {
         LogTools.info("websocket onOpen");
         websocket.sendTextFrame("HELLO 852978");
      }

      @Override
      public void onClose(WebSocket websocket, int code, String reason)
      {
         LogTools.info("WebSocket onClose: {}: {}", code, reason);
         endCall();
      }

      @Override
      public void onTextFrame(String payload, boolean finalFragment, int rsv)
      {
         if (payload.equals("HELLO"))
         {
            websocket.sendTextFrame("SESSION " + SESSION_ID);
         }
         else if (payload.equals("SESSION_OK"))
         {
            pipeline.play();
         }
         else if (payload.startsWith("ERROR"))
         {
            LogTools.error(payload);
            endCall();
         }
         else
         {
            handleSDP(payload);
         }
      }

      @Override
      public void onError(Throwable throwable)
      {
         throwable.printStackTrace();
      }
   };

   // Session Description Protocol (SDP)
   private void handleSDP(String payload)
   {
      try
      {
         JsonNode answer = mapper.readTree(payload);
         if (answer.has("sdp"))
         {
            String sdpStr = answer.get("sdp").get("sdp").textValue();
            LogTools.info("Answer SDP:\n{}", sdpStr);
            SDPMessage sdpMessage = new SDPMessage();
            sdpMessage.parseBuffer(sdpStr);
            WebRTCSessionDescription description = new WebRTCSessionDescription(WebRTCSDPType.ANSWER, sdpMessage);
            webRTCBin.setRemoteDescription(description);
         }
         else if (answer.has("ice"))
         {
            String candidate = answer.get("ice").get("candidate").textValue();
            int sdpMLineIndex = answer.get("ice").get("sdpMLineIndex").intValue();
            LogTools.info("Adding ICE candidate: {}", candidate);
            webRTCBin.addIceCandidate(sdpMLineIndex, candidate);
         }
      }
      catch (IOException ioException)
      {
         ioException.printStackTrace();
      }
   }

   private void setupPipeLogging(Pipeline pipe)
   {
      Bus bus = pipe.getBus();
      bus.connect((Bus.EOS) source ->
      {
         LogTools.info("Reached end of stream: {}", source.toString());
         endCall();
      });
      bus.connect((Bus.ERROR) (source, code, message) ->
      {
         LogTools.error("Error from source: {}, with code: {}, and message: {}", source, code, message);
         endCall();
      });
      bus.connect((source, old, current, pending) ->
      {
         if (source instanceof Pipeline)
         {
            LogTools.info("Pipe state changed from {} to {}", old, current);
         }
      });
   }

   private final WebRTCBin.CREATE_OFFER onOfferCreated = offer ->
   {
      webRTCBin.setLocalDescription(offer);
      try
      {
         ObjectNode rootNode = mapper.createObjectNode();
         ObjectNode sdpNode = mapper.createObjectNode();
         sdpNode.put("type", "offer");
         sdpNode.put("sdp", offer.getSDPMessage().toString());
         rootNode.set("sdp", sdpNode);
         String json = mapper.writeValueAsString(rootNode);
         LogTools.info("Sending offer:\n{}", json);
         websocket.sendTextFrame(json);
      }
      catch (JsonProcessingException e)
      {
         e.printStackTrace();
      }
   };

   private final WebRTCBin.ON_NEGOTIATION_NEEDED onNegotiationNeeded = elem ->
   {
      LogTools.info(() -> "onNegotiationNeeded: " + elem.getName());

      // When webrtcbin has created the offer, it will hit our callback and we
      // send SDP offer over the websocket to signalling server
      webRTCBin.createOffer(onOfferCreated);
   };

   private final WebRTCBin.ON_ICE_CANDIDATE onIceCandidate = (sdpMLineIndex, candidate) ->
   {
      ObjectNode rootNode = mapper.createObjectNode();
      ObjectNode iceNode = mapper.createObjectNode();
      iceNode.put("candidate", candidate);
      iceNode.put("sdpMLineIndex", sdpMLineIndex);
      rootNode.set("ice", iceNode);

      try
      {
         String json = mapper.writeValueAsString(rootNode);
         LogTools.info(() -> "ON_ICE_CANDIDATE: " + json);
         websocket.sendTextFrame(json);
      }
      catch (JsonProcessingException jsonProcessingException)
      {
         jsonProcessingException.printStackTrace();
      }
   };

   private final Element.PAD_ADDED onDecodedStream = (element, pad) ->
   {
      if (!pad.hasCurrentCaps())
      {
         LogTools.info("Pad has no current Capabilities - ignoring");
         return;
      }
      Caps caps = pad.getCurrentCaps();
      LogTools.info("Received decoded stream with capabilities: {}", caps.toString());
      if (caps.isAlwaysCompatible(Caps.fromString("video/x-raw")))
      {
         Element q = ElementFactory.make("queue", "videoqueue");
         Element conv = ElementFactory.make("videoconvert", "videoconvert");
         Element sink = ElementFactory.make("autovideosink", "videosink");
         pipeline.addMany(q, conv, sink);
         q.syncStateWithParent();
         conv.syncStateWithParent();
         sink.syncStateWithParent();
         pad.link(q.getStaticPad("sink"));
         q.link(conv);
         conv.link(sink);
      }
      else if (caps.isAlwaysCompatible(Caps.fromString("audio/x-raw")))
      {
         Element q = ElementFactory.make("queue", "audioqueue");
         Element conv = ElementFactory.make("audioconvert", "audioconvert");
         Element resample = ElementFactory.make("audioresample", "audioresample");
         Element sink = ElementFactory.make("autoaudiosink", "audiosink");
         pipeline.addMany(q, conv, resample, sink);
         q.syncStateWithParent();
         conv.syncStateWithParent();
         resample.syncStateWithParent();
         sink.syncStateWithParent();
         pad.link(q.getStaticPad("sink"));
         q.link(conv);
         conv.link(resample);
         resample.link(sink);
      }
   };

   private final Element.PAD_ADDED onIncomingStream = (element, pad) ->
   {
      LogTools.info("Receiving stream! Element: {} Pad: {}", element.getName(), pad.getName());
      if (pad.getDirection() != PadDirection.SRC)
      {
         return;
      }
      DecodeBin decodeBin = new DecodeBin("decodebin_" + pad.getName());
      decodeBin.connect(onDecodedStream);
      pipeline.add(decodeBin);
      decodeBin.syncStateWithParent();
      pad.link(decodeBin.getStaticPad("sink"));
   };

   public static void main(String[] args)
   {
      new GDXWebcamGStreamerWebRTCSender();
   }
}
