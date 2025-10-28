package org.ros.message;

public interface MessageFactory {
   <T> T newFromType(String var1);
}