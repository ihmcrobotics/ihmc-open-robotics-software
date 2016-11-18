package us.ihmc.humanoidRobotics.communication.packets.sensing;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;

import java.util.Random;

/**
 * Created by adrien on 11/16/16.
 */
public class REAMessagePacket extends Packet<REAMessagePacket> {


    private String messageName;
    private Object messageContent; // TODO try to narrow down the type of the object

    public REAMessagePacket(Random random)
    {

    }

    public REAMessagePacket()
    {
        setDestination(PacketDestination.REA_MODULE); // or use broadcast
    }


    public REAMessagePacket(String messageName, Object messageContent)
    {
        setDestination(PacketDestination.REA_MODULE); // or use broadcast
        setMessageName(messageName);
        setMessageContent(messageContent);
    }


    @Override
    public boolean epsilonEquals(REAMessagePacket other, double epsilon) {


        return false;
    }

    // TODO Setters and getters for message and its content


    public void setMessageName(String messageName) {
        this.messageName = messageName;
    }

    public void setMessageContent(Object content) {
        this.messageContent = content;
    }

    public String getMessageName() {
        return messageName;
    }

    public Object getMessageContent() {
        return messageContent;
    }
}
