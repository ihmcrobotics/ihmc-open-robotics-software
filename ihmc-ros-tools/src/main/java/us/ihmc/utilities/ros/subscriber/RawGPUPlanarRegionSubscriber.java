package us.ihmc.utilities.ros.subscriber;

import map_sense.RawGPUPlanarRegionList;
import us.ihmc.log.LogTools;
import us.ihmc.utilities.ros.RosMainNode;

import java.net.URI;
import java.net.URISyntaxException;

public class RawGPUPlanarRegionSubscriber extends AbstractRosTopicSubscriber<map_sense.RawGPUPlanarRegionList>{

    public RawGPUPlanarRegionSubscriber() {
        super(RawGPUPlanarRegionList._TYPE);
    }

    @Override
    public void onNewMessage(RawGPUPlanarRegionList message) {
        System.out.println("Received Message:" + message.getHeader());
    }

    public static void main(String[] args) throws URISyntaxException {
        URI rosMasterURI = new URI("http://localhost:11311/");
        RosMainNode rosMainNode = new RosMainNode(rosMasterURI, "GPUPlanarRegionSubscriber");
        RawGPUPlanarRegionSubscriber subscriber = new RawGPUPlanarRegionSubscriber();
        rosMainNode.attachSubscriber("/map/regions/test", subscriber);
        rosMainNode.execute();
    }
}
