package us.ihmc.utilities.ros.subscriber;

import map_sense.RawGPUPlanarRegion;
import map_sense.RawGPUPlanarRegionList;
import us.ihmc.utilities.ros.RosMainNode;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.List;

public class RawGPUPlanarRegionSubscriber extends AbstractRosTopicSubscriber<map_sense.RawGPUPlanarRegionList> {

    private RawGPUPlanarRegionList planarRegionList;
    private boolean regionListAvailable = false;

    public boolean regionListIsAvailable() {
        return regionListAvailable;
    }

    public RawGPUPlanarRegionList getPlanarRegionList() {
        this.regionListAvailable = false;
        return planarRegionList;
    }

    public RawGPUPlanarRegionSubscriber() {
        super(RawGPUPlanarRegionList._TYPE);
    }

    @Override
    public void onNewMessage(RawGPUPlanarRegionList message) {
        this.planarRegionList = message;
        this.regionListAvailable = true;
        System.out.println("Received Message:" + message.getNumOfRegions());
        for (int i = 0; i < message.getNumOfRegions(); i++) {
            List<RawGPUPlanarRegion> regions = message.getRegions();
            System.out.println("\tRegion:" + regions.get(i).getId());
            System.out.println("\t\tNormal:" + "X:" + regions.get(i).getNormal().getX() + "\tY:" + regions.get(i).getNormal().getY() + "\tZ:" + regions.get(i).getNormal().getZ());
            System.out.println("\t\tCentroid:" + "X:" + regions.get(i).getCentroid().getX() + "\tY:" + regions.get(i).getCentroid().getY() + "\tZ:" + regions.get(i).getCentroid().getZ());
//            for (int j = 0; j < regions.get(i).getVertices().size(); j++) {
//                geometry_msgs.Point point = regions.get(i).getVertices().get(j);
//                System.out.println("\t\tPoint:" + "X:" + point.getX() + "\tY:" + point.getY() + "\tZ:" + point.getZ());
//            }
        }
    }

    public static void main(String[] args) throws URISyntaxException {
        URI rosMasterURI = new URI("http://localhost:11311/");
        RosMainNode rosMainNode = new RosMainNode(rosMasterURI, "GPUPlanarRegionSubscriber");
        RawGPUPlanarRegionSubscriber subscriber = new RawGPUPlanarRegionSubscriber();
        rosMainNode.attachSubscriber("/map/regions/test", subscriber);
        rosMainNode.execute();
    }
}
